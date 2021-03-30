/*
 * Copyright 2021 Ian Pilcher <arequipeno@gmail.com>
 *
 * The program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#define _GNU_SOURCE

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <limits.h>
#include <signal.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <atasmart.h>
#include <freeipmi/freeipmi.h>
#include <yaml.h>

/*
 * https://forums.servethehome.com/index.php?resources/supermicro-x9-x10-x11-fan-speed-control.20/
 * https://www.supermicro.com/support/faqs/faq.cfm?faq=31537
 */

#define SMFD_SUPERMICRO_IPMI_CMD_FAN_MODE	0x45
#define SMFD_SUPERMICRO_IPMI_EXT_FAN_PERCENT	0x66
#define SMFD_FAN_ZONE_CPU			0x00
#define SMFD_FAN_ZONE_SYS			0x01
#define SMFD_SUPERMICRO_FAN_MODE_STD		0x00
#define SMFD_SUPERMICRO_FAN_MODE_FULL		0x01
#define SMFD_SUPERMICRO_FAN_MODE_OPT		0x02
#define SMFD_SUPERMICRO_FAN_MODE_IO		0x04


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	Data types
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* A temperature which triggers minimum fan percentages */
struct smfd_temp_threshold {
	char *name;
	int threshold;
	int hysteresis;
	uint8_t	cpu_fan_percent;
	uint8_t sys_fan_percent;
	_Bool active;
};

/* Minimum fan percentages after processing all thresholds for a temperature */
struct smfd_process_temp_result {
	struct smfd_temp_threshold *cpu_threshold;
	struct smfd_temp_threshold *sys_threshold;
	uint8_t	cpu_fan_percent;
	uint8_t sys_fan_percent;
	char name[sizeof "system"];
};

/* Used to read & store 1 fan RPM via IPMI */
struct smfd_ipmi_fan {
	char *name;
	unsigned int rpm;
	unsigned int record_len;
	uint16_t record_id;
	uint8_t record[IPMI_SDR_MAX_RECORD_LENGTH];
};

/* A single temperature reading and associated periodic info */
struct smfd_temperature {
	int current;		/* most recent reading */
	int high;		/* highest reading in sample period */
	int low;		/* lowest reading in sample period */
	int accumulator;	/* total of all readings in sample period */
	int samples;		/* number of readings in sample period */
};

/* Used to read & store 1 temperature from the coretemp module */
struct smfd_coretemp {
	char *name;
	FILE *fp;
	struct smfd_temperature temp;
};

/* Used to read & store 1 disk temperature via S.M.A.R.T. */
struct smfd_disk {
	char *name;
	SkDisk *disk;
	struct smfd_temperature temp;
};


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	Command line options, configuration & global state
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Log to syslog (instead of stderr)? */
static _Bool smfd_use_syslog = 0;

/* Log debug messages? */
static _Bool smfd_debug = 0;

/* Dump configuration & exit? */
static _Bool smfd_config_test = 0;

/* Configuration file */
static const char *smfd_config_file = "/etc/smfd/config.yaml";

/* How often to log temperature & other information (seconds) */
static unsigned int smfd_log_interval = UINT_MAX;

/* IPMI SDR cache location */
static char smfd_sdr_cache_default[] = "/var/lib/smfd/sdr-cache";
static char *smfd_sdr_cache = smfd_sdr_cache_default;

/* Fan percent settings when no thresholds are triggered */
static uint8_t smfd_cpu_fan_base = 255;
static uint8_t smfd_sys_fan_base = 255;

/* CPU temperature (package temperature or any core) thresholds */
static struct smfd_temp_threshold *smfd_cfg_cpu_temp = NULL;

/* PCH temperature thresholds */
static struct smfd_temp_threshold *smfd_cfg_pch_temp = NULL;

/* Disk temperature thresholds */
static struct smfd_temp_threshold *smfd_cfg_disk_temp = NULL;

/* IPMI fans */
static struct smfd_ipmi_fan *smfd_ipmi_fans = NULL;
static unsigned int smfd_ipmi_fan_count = 0;

/* CPU package & core temperatures */
static struct smfd_coretemp *smfd_coretemps;
static unsigned int smfd_coretemp_count;

/* S.M.A.R.T. disk temperatures */
static struct smfd_disk *smfd_disks = NULL;
static unsigned int smfd_disk_count = 0;

/* PCH temperature */
static struct smfd_temperature smfd_pch_temp;

/* FreeIPMI "context" for IPMI commands */
static ipmi_ctx_t smfd_ipmi = NULL;

/* FreeIPMI "context" for reading sensors values */
static ipmi_sensor_read_ctx_t smfd_read = NULL;

/* Used to read PCH temperature */
static FILE *smfd_pch_temp_fp = NULL;

/* Current CPU fan percentage */
static uint8_t smfd_cpu_fan_percent = 100;

/* Current system fan percentage */
static uint8_t smfd_sys_fan_percent = 100;

/* Signal flags */
static volatile sig_atomic_t smfd_debug_signal = 0;	/* SIGUSR1 */
static volatile sig_atomic_t smfd_dump_signal = 0;	/* SIGUSR2 */
static volatile sig_atomic_t smfd_quit_signal = 0;	/* SIGTERM or SIGINT */

/* Time at which to log temperature & other data */
static time_t smfd_next_log;

/* Time at which data collection started */
static time_t smfd_log_start;


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	Logging/debugging
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Log a message to syslog or stderr */
__attribute__((format(printf, 2, 3)))
static void smfd_log(const int level, const char *const format, ...)
{
	va_list ap;

	va_start(ap, format);

	if (smfd_use_syslog)
		vsyslog(level, format, ap);
	else
		vfprintf(stderr, format, ap);

	va_end(ap);
}

/* Preprocessor dance to "stringify" an expanded macro value (e.g. __LINE__) */
#define SMFD_STR_RAW(x)		#x
#define SMFD_STR(x)		SMFD_STR_RAW(x)

/* Expands to a message preamble which specifies file & line */
#define SMFD_LOCATION		__FILE__ ":" SMFD_STR(__LINE__) ": "

/* Expands to syslog priority & full message preamble */
#define SMFD_LOG_HDR(l)		LOG_ ## l, #l ": " SMFD_LOCATION

/* Debug messages are logged at INFO priority to avoid syslog filtering */
#define SMFD_DEBUG(...)										\
	do {											\
		if (!smfd_debug)								\
			break;									\
		smfd_log(LOG_INFO, "DEBUG: " SMFD_LOCATION __VA_ARGS__);			\
	}											\
	while (0)

/* Print/log a message at the given priority */
#define SMFD_INFO(...)		smfd_log(SMFD_LOG_HDR(INFO) __VA_ARGS__)
#define SMFD_NOTICE(...)	smfd_log(SMFD_LOG_HDR(NOTICE) __VA_ARGS__)
#define SMFD_WARNING(...)	smfd_log(SMFD_LOG_HDR(WARNING) __VA_ARGS__)
#define SMFD_ERR(...)		smfd_log(SMFD_LOG_HDR(ERR) __VA_ARGS__)
#define SMFD_CRIT(...)		smfd_log(SMFD_LOG_HDR(CRIT) __VA_ARGS__)
#define SMFD_ALERT(...)		smfd_log(SMFD_LOG_HDR(ALERT) __VA_ARGS__)
#define SMFD_EMERG(...)		smfd_log(SMFD_LOG_HDR(EMERG) __VA_ARGS__)

/* Print/log an unexpected internal error and abort */
#define SMFD_ABORT(...)		do { SMFD_CRIT(__VA_ARGS__); abort(); } while (0)

/* Print a fatal error and exit immediately */
#define SMFD_FATAL(...)		do { SMFD_ERR(__VA_ARGS__); exit(EXIT_FAILURE); } while (0)

/* LibYAML doesn't seem to provide any sort of error strings */
static const char *smfd_libyaml_errmsg(const yaml_error_type_t err)
{
	switch (err) {

		case YAML_NO_ERROR:		return "LibYAML SUCCESS";
		case YAML_MEMORY_ERROR:		return "LibYAML memory error";
		case YAML_READER_ERROR:		return "LibYAML reader error";
		case YAML_SCANNER_ERROR:	return "LibYAML scanner error";
		case YAML_PARSER_ERROR:		return "LibYAML parser error";
		case YAML_COMPOSER_ERROR:	return "LibYAML composer error";
		case YAML_WRITER_ERROR:		return "LibYAML writer error";
		case YAML_EMITTER_ERROR:	return "LibYAML emitter error";
		default:			return "LibYAML UNKNOWN error";
	}
}

/* LibYAML's error reporting isn't really documented; do the best we can */
#define SMFD_LIBYAML_FATAL(f, p, ...)								\
	do {											\
		if ((p)->error >= YAML_READER_ERROR && (p)->error <= YAML_EMITTER_ERROR) {	\
			if ((p)->problem == NULL)						\
				SMFD_FATAL("%s", smfd_libyaml_errmsg((p)->error));		\
			SMFD_FATAL("%s:%zd:%zd: %s\n", (f), (p)->problem_mark.line + 1,		\
				   (p)->problem_mark.column + 1, (p)->problem);			\
		}										\
												\
		SMFD_ABORT("%s", smfd_libyaml_errmsg((p)->error));				\
	}											\
	while (0)

/* Fatal config file error with location */
#define SMFD_CFG_FATAL(s, n, ...)								\
	SMFD_FATAL("Invalid configuration: %s:%zd:%zd: " s, smfd_config_file,			\
		   (n)->start_mark.line + 1, (n)->start_mark.column + 1, ##__VA_ARGS__)

/* Prepare a temperature for a new logging period */
static void smfd_temp_reset(struct smfd_temperature *const temp)
{
	temp->high = INT_MIN;
	temp->low = INT_MAX;
	temp->accumulator = 0;
	temp->samples = 0;
}

/* Update a temperature with a new current reading */
static void smfd_update_temp(struct smfd_temperature *const temp, const int current)
{
	temp->current = current;

	if (current > temp->high)
		temp->high = current;

	if (current < temp->low)
		temp->low = current;

	temp->accumulator += current;
	temp->samples += 1;
}

/* Log and reset information about 1 temperature */
static void smfd_log_temp(const char *const name, struct smfd_temperature *const temp)
{
	SMFD_INFO("%s: current: %d°C, high: %d°C, low: %d°C, mean: %d°C\n", name,
		  temp->current, temp->high, temp->low,
		  (temp->accumulator + temp->samples / 2) / temp->samples);

	smfd_temp_reset(temp);
}

/* Forward declarations needed by smfd_log_info */
static uint8_t smfd_get_fan_mode(void);
static uint8_t smfd_get_fan_percent(uint8_t zone);
static void smfd_ipmi_fan_read(void);

/* Log the current system state and some periodic statistics */
static void smfd_log_info(void)
{
	static const char *const fan_modes[] = {
		[SMFD_SUPERMICRO_FAN_MODE_STD]	= "Standard",			/* 0x00 */
		[SMFD_SUPERMICRO_FAN_MODE_FULL]	= "Full Speed (manual)",	/* 0x01 */
		[SMFD_SUPERMICRO_FAN_MODE_OPT]	= "Optimal",			/* 0x02 */
		[3]				= "UNKNOWN",			/* 0x03 */
		[SMFD_SUPERMICRO_FAN_MODE_IO]	= "Heavy I/O"			/* 0x04 */
	};

	uint8_t fan_mode, cpu_fan_speed, sys_fan_speed;
	unsigned int i;

	/* This is the only time that IPMI information is read */
	fan_mode = smfd_get_fan_mode();
	cpu_fan_speed = smfd_get_fan_percent(SMFD_FAN_ZONE_CPU);
	sys_fan_speed = smfd_get_fan_percent(SMFD_FAN_ZONE_SYS);
	smfd_ipmi_fan_read();

	SMFD_INFO("Data collection began at %s", ctime(&smfd_log_start));

	SMFD_INFO("BMC fan mode: %s\n",
		  (fan_mode <= SMFD_SUPERMICRO_FAN_MODE_IO) ? fan_modes[fan_mode] : "UNKNOWN");
	SMFD_INFO("CPU fan duty cycle: %" PRIu8 "%%\n", cpu_fan_speed);
	SMFD_INFO("System fan duty cycle: %" PRIu8 "%%\n", sys_fan_speed);

	for (i = 0; i < smfd_ipmi_fan_count; ++i)
		SMFD_INFO("%s: %u RPM\n", smfd_ipmi_fans[i].name, smfd_ipmi_fans[i].rpm);

	smfd_log_temp("PCH", &smfd_pch_temp);

	for (i = 0; i < smfd_coretemp_count; ++i)
		smfd_log_temp(smfd_coretemps[i].name, &smfd_coretemps[i].temp);

	for (i = 0; i < smfd_disk_count; ++i)
		smfd_log_temp(smfd_disks[i].name, &smfd_disks[i].temp);
}


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	Command line parsing
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

static void smfd_parse_args(const int argc, char **const argv)
{
	static const char help_msg[] =
			"Usage: %s [-h|--help]\n"
			"       %s [-d] [-s] [-c CONFIG_FILE ]\n"
			"\n"
			"  -h, --help        show this message and exit\n"
			"  -d                print/log debugging messages\n"
			"  -s                log to syslog (when running in a terminal)\n"
			"  -p                print/log configuration & exit (implies -d)\n"
			"  -c CONFIG_FILE    configuration file [/etc/smfd/config.yaml]\n";

	int i;

	for (i = 1; i < argc; ++i) {
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf(help_msg, argv[0], argv[0]);
			exit(EXIT_SUCCESS);
		}
	}

	smfd_use_syslog = !isatty(STDERR_FILENO);

	for (i = 1; i < argc; ++i) {

		if (strcmp(argv[i], "-d") == 0) {
			smfd_debug = 1;
			continue;
		}

		if (strcmp(argv[i], "-s") == 0) {
			smfd_use_syslog = 1;
			continue;
		}

		if (strcmp(argv[i], "-p") == 0) {
			smfd_config_test = 1;
			smfd_debug = 1;
			continue;
		}

		if (strcmp(argv[i], "-c") == 0) {
			if ((smfd_config_file = argv[++i]) == NULL)
				SMFD_FATAL("-c option requires configuration file\n");
			continue;
		}
	}
}


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	coretemp & PCH temperatures
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Initialize smfd_coretemps with the name of each coretemp input and an open stream to each */
static void smfd_coretemp_init(void)
{
	static const char hwmon_dir[] = "/sys/devices/platform/coretemp.0/hwmon/hwmon2";

	char buf[sizeof "tempXX_input"];
	int dirfd, rc, fd;
	size_t label_size;
	char *labels[99];
	unsigned i;
	FILE *fp;

	if ((dirfd = open(hwmon_dir, O_DIRECTORY | O_PATH)) < 0)
		SMFD_FATAL("%s: %m\n", hwmon_dir);

	for (i = 0; i < 99; ++i) {

		if ((rc = snprintf(buf, sizeof buf, "temp%u_label", i + 1)) < 0)
			SMFD_ABORT("snprintf: %m\n");

		if (rc >= (int)sizeof buf)
			SMFD_FATAL("File name truncated: temp%u_label", i + 1);

		if ((fd = openat(dirfd, buf, O_RDONLY)) < 0) {
			if (errno == ENOENT)
				break;
			SMFD_FATAL("%s/%s: %m\n", hwmon_dir, buf);
		}

		if ((fp = fdopen(fd, "r")) == NULL)
			SMFD_ABORT("fdopen: %m\n");

		labels[i] = NULL;
		label_size = 0;

		if ((rc = getline(&labels[i], &label_size, fp)) < 0)
			SMFD_FATAL("getline: %m\n");

		assert(rc >= (int)sizeof "Core 0");

		if (labels[i][rc - 1] == '\n')
			labels[i][rc - 1] = 0;

		if (fclose(fp) != 0)
			SMFD_FATAL("fclose: %m\n");
	}

	if (i == 0)
		SMFD_FATAL("No temperature inputs found in %s\n", hwmon_dir);

	smfd_coretemp_count = i;
	SMFD_DEBUG("Found %u coretemp inputs\n", i);

	if ((smfd_coretemps = malloc(smfd_coretemp_count * sizeof *smfd_coretemps)) == NULL)
		SMFD_ABORT("malloc: %m\n");

	for (i = 0; i < smfd_coretemp_count; ++i) {

		smfd_coretemps[i].name = labels[i];
		smfd_temp_reset(&smfd_coretemps[i].temp);

		if ((rc = snprintf(buf, sizeof buf, "temp%u_input", i + 1)) < 0)
			SMFD_ABORT("snprintf: %m\n");

		if (rc >= (int) sizeof buf)
			SMFD_FATAL("File name truncated: temp%u_input\n", i + 1);

		if ((fd = openat(dirfd, buf, O_RDONLY)) < 0)
			SMFD_FATAL("%s/%s: %m\n", hwmon_dir, buf);

		if ((smfd_coretemps[i].fp = fdopen(fd, "r")) == NULL)
			SMFD_ABORT("fdopen: %m\n");

		if (setvbuf(smfd_coretemps[i].fp, NULL, _IONBF, 0) != 0)
			SMFD_ABORT("setvbuf: %m\n");
	}

	if (close(dirfd) != 0)
		SMFD_FATAL("close: %m\n");

	SMFD_DEBUG("smfd_coretemp_init finished\n");
}

/* Initialize smfd_pch_temp_fp with an open stream to the PCH temperature input */
static void smfd_pch_temp_init(void)
{
	static const char input[] = "/sys/devices/virtual/thermal/thermal_zone0/hwmon0/temp1_input";

	smfd_temp_reset(&smfd_pch_temp);

	if ((smfd_pch_temp_fp = fopen(input, "r")) == NULL)
		SMFD_FATAL("%s: %m\n", input);

	if (setvbuf(smfd_pch_temp_fp, NULL, _IONBF, 0) != 0)
		SMFD_ABORT("setvbuf: %m\n");

	SMFD_DEBUG("smfd_pch_temp_init finished\n");
}

/* Clean up smfd_coretemps - close streams & free memory */
static void smfd_coretemp_fini(void)
{
	unsigned i;

	for (i = 0; i < smfd_coretemp_count; ++i) {

		free(smfd_coretemps[i].name);

		if (fclose(smfd_coretemps[i].fp) != 0)
			SMFD_ERR("fclose: %m\n");
	}

	free(smfd_coretemps);
}

/* Close smfd_pch_temp_fp */
static void smfd_pch_temp_fini(void)
{
	if (fclose(smfd_pch_temp_fp) != 0)
		SMFD_ERR("fclose: %m\n");
}

/* Read & parse a coretemp or PCH temperature */
static void smfd_temp_read(FILE *const fp, const char *const name,
			   struct smfd_temperature *const temp)
{
	int rc, reading;

	rewind(fp);

	if ((rc = fscanf(fp, "%d", &reading)) == EOF)
		SMFD_FATAL("%s: %m\n", name);

	if (rc != 1)
		SMFD_FATAL("Failed to parse %s temperature\n", name);

	smfd_update_temp(temp, (reading + 500) / 1000);

	if (temp->current < 0 || temp->current > 120)
		SMFD_WARNING("%s reading (%d°C) is probably garbage\n", name, temp->current);
}

/* Read & parse the temperature from every coretemp input */
static void smfd_coretemp_read(void)
{
	unsigned i;

	for (i = 0; i < smfd_coretemp_count; ++i) {

		smfd_temp_read(smfd_coretemps[i].fp, smfd_coretemps[i].name,
			       &smfd_coretemps[i].temp);
	}
}

/* Read & parse the PCH temperature */
static void smfd_pch_temp_read(void)
{
	smfd_temp_read(smfd_pch_temp_fp, "PCH", &smfd_pch_temp);
}


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	S.M.A.R.T. disk temperatures
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Initialize smfd_disks with a libatasmart "handle" for each disk */
static void smfd_disk_init(void)
{
	unsigned i;

	for (i = 0; i < smfd_disk_count; ++i) {

		smfd_temp_reset(&smfd_disks[i].temp);

		if (sk_disk_open(smfd_disks[i].name, &smfd_disks[i].disk) < 0)
			SMFD_FATAL("%s: %m\n", smfd_disks[i].name);
	}

	SMFD_DEBUG("smfd_disk_init finished\n");
}

/* Close the libatasmart handle for each disk in smfd_disks and free memory */
static void smfd_disk_fini(void)
{
	unsigned i;

	for (i = 0; i < smfd_disk_count; ++i) {
		sk_disk_free(smfd_disks[i].disk);
		free(smfd_disks[i].name);
	}

	free(smfd_disks);
}

/* Read the temperature of each disk in smfd_disks */
static void smfd_disk_read(void)
{
	uint64_t mkelvin;
	unsigned i;

	for (i = 0; i < smfd_disk_count; ++i) {

		if (sk_disk_smart_read_data(smfd_disks[i].disk) < 0)
			SMFD_FATAL("%s: %m\n", smfd_disks[i].name);

		if (sk_disk_smart_get_temperature(smfd_disks[i].disk, &mkelvin) < 0)
			SMFD_FATAL("%s: %m\n", smfd_disks[i].name);

		if (mkelvin > (uint64_t)INT_MAX) {
			SMFD_FATAL("%s: temperature (%" PRIu64 ") out of range\n",
				   smfd_disks[i].name, mkelvin);
		}

		/* Absolute zero == -273.15°C */
		smfd_update_temp(&smfd_disks[i].temp ,(mkelvin - 273150 + 500) / 1000);
	}
}


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	Raw IPMI commands to query/set BMC fan mode & fan zone speeds (percentages)
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Send a raw command to the BMC; check for success and a response of the expected length */
static void smfd_ipmi_raw_cmd(const uint8_t *const cmd, const unsigned int cmd_len,
			      uint8_t *const restrict response, const unsigned int response_len)
{
	static const uint8_t net_fn = IPMI_NET_FN_OEM_SUPERMICRO_GENERIC_RQ;	/* 0x30 */

	char msg[IPMI_ERR_STR_MAX_LEN];
	uint8_t resp[256];
	int rc;

	memset(resp, 0, sizeof resp);

	if ((rc = ipmi_cmd_raw(smfd_ipmi, 0, net_fn, cmd, cmd_len, resp, sizeof resp)) < 0)
		SMFD_FATAL("ipmi_cmd_raw: %s\n", ipmi_ctx_errormsg(smfd_ipmi));

	if (rc < 2)
		SMFD_FATAL("Truncated(?) IPMI response (%d bytes)\n", rc);

	if (resp[1] != IPMI_COMP_CODE_COMMAND_SUCCESS) {
		if (ipmi_completion_code_strerror_r(resp[0], net_fn, resp[1], msg, sizeof msg) < 0)
			snprintf(msg, sizeof msg, "[completion code %0x" PRIx8 "]", resp[1]);
		SMFD_FATAL("IPMI command failed: %s\n", msg);
	}

	if (resp[0] != cmd[0]) {
		SMFD_FATAL("IPMI response (0x%" PRIx8 ") did not match command (0x%" PRIx8 ")\n",
			   resp[0], cmd[0]);
	}

	if ((unsigned int)rc != response_len + 2) {
		SMFD_FATAL("Unexpected response data size (got %d bytes, expected %u)\n",
			   rc - 2, response_len);
	}

	if (response_len > 0)
		memcpy(response, resp + 2, response_len);
}

/* Query the current BMC fan management mode */
static uint8_t smfd_get_fan_mode(void)
{
	static const uint8_t cmd[] = {
		SMFD_SUPERMICRO_IPMI_CMD_FAN_MODE,
		0x00
	};

	uint8_t mode;

	smfd_ipmi_raw_cmd(cmd, sizeof cmd, &mode, sizeof mode);

	return mode;
}

/* Set the BMC fan management mode */
static void smfd_set_fan_mode(const uint8_t mode)
{
	static uint8_t cmd[] = {
		SMFD_SUPERMICRO_IPMI_CMD_FAN_MODE,
		0x01,
		0x00	/* mode goes here */
	};

	cmd[2] = mode;
	smfd_ipmi_raw_cmd(cmd, sizeof cmd, NULL, 0);
}

/* Query the current fan duty cycle (percentage) of a zone */
static uint8_t smfd_get_fan_percent(const uint8_t zone)
{
	uint8_t cmd[] = {
		IPMI_CMD_OEM_SUPERMICRO_GENERIC_EXTENSION,
		SMFD_SUPERMICRO_IPMI_EXT_FAN_PERCENT,
		0x00,
		0x00	/* zone goes here */
	};

	uint8_t percent;

	cmd[3] = zone;
	smfd_ipmi_raw_cmd(cmd, sizeof cmd, &percent, sizeof percent);

	return percent;
}

/* Set the fan duty cycle (percentage) of a zone */
static void smfd_set_fan_percent(const uint8_t zone, const uint8_t percent)
{
	uint8_t cmd[] = {
		IPMI_CMD_OEM_SUPERMICRO_GENERIC_EXTENSION,
		SMFD_SUPERMICRO_IPMI_EXT_FAN_PERCENT,
		0x01,
		0x00,	/* zone goes here */
		0x00	/* percent goes here */
	};

	cmd[3] = zone;
	cmd[4] = percent;
	smfd_ipmi_raw_cmd(cmd, sizeof cmd, NULL, 0);
}


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	IPMI initialization & fan sensor reading
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Initialize a smfd_ipmi_fan by reading its record from the IPMI SDR cache */
static void smfd_ipmi_fan_init(const ipmi_sdr_ctx_t sdr, struct smfd_ipmi_fan *const fan)
{
	uint8_t record_type, sensor_type;
	uint16_t record_id;
	int rc;

	if (ipmi_sdr_cache_search_record_id(sdr, fan->record_id) < 0)
		SMFD_FATAL("ipmi_sdr_cache_search_record_id: %s\n", ipmi_sdr_ctx_errormsg(sdr));

	if (ipmi_sdr_parse_record_id_and_type(sdr, NULL, 0, &record_id, &record_type) < 0)
		SMFD_FATAL("ipmi_sdr_parse_record_id_and_type: %s\n", ipmi_sdr_ctx_errormsg(sdr));

	assert(record_id == fan->record_id);

	if (record_type != IPMI_SDR_FORMAT_FULL_SENSOR_RECORD)
		SMFD_FATAL("%s [%" PRIu16 "] is not a full sensor record\n", fan->name, record_id);

	if (ipmi_sdr_parse_sensor_type(sdr, NULL, 0, &sensor_type) < 0)
		SMFD_FATAL("ipmi_sdr_parse_sensor_type: %s\n", ipmi_sdr_ctx_errormsg(sdr));

	if (sensor_type != IPMI_SENSOR_TYPE_FAN)
		SMFD_FATAL("%s [%" PRIu16 "] is not a fan sensor\n", fan->name, record_id);

	if ((rc = ipmi_sdr_cache_record_read(sdr, fan->record, sizeof fan->record)) < 0)
		SMFD_FATAL("ipmi_sdr_cache_record_read: %s\n", ipmi_sdr_ctx_errormsg(sdr));

	fan->record_len = rc;
}

/* Initialize smfd_ipmi, smfd_ipmi_fans & smfd_read; set fan mode to full & set all fans to 100% */
static void smfd_ipmi_init(void)
{
	ipmi_sdr_ctx_t sdr;
	unsigned i;
	int rc;

	if ((smfd_ipmi = ipmi_ctx_create()) == NULL)
		SMFD_ABORT("ipmi_ctx_create: %m\n");

	if ((rc = ipmi_ctx_find_inband(smfd_ipmi, NULL, 0, 0, 0, NULL, 0, 0)) < 0)
		SMFD_FATAL("ipmi_ctx_find_inband: %s\n", ipmi_ctx_errormsg(smfd_ipmi));

	if (rc == 0)
		SMFD_FATAL("Could not find in-band IPMI device\n");

	if ((sdr = ipmi_sdr_ctx_create()) == NULL)
		SMFD_ABORT("ipmi_sdr_ctx_create: %m\n");

	if (ipmi_sdr_cache_open(sdr, smfd_ipmi, smfd_sdr_cache) < 0)
		SMFD_FATAL("ipmi_sdr_cache_open: %s\n", ipmi_sdr_ctx_errormsg(sdr));

	for (i = 0; i < smfd_ipmi_fan_count; ++i)
		smfd_ipmi_fan_init(sdr, &smfd_ipmi_fans[i]);

	if (ipmi_sdr_cache_close(sdr) < 0)
		SMFD_ERR("ipmi_sdr_cache_close: %s\n", ipmi_sdr_ctx_errormsg(sdr));

	ipmi_sdr_ctx_destroy(sdr);

	if ((smfd_read = ipmi_sensor_read_ctx_create(smfd_ipmi)) == NULL)
		SMFD_FATAL("ipmi_sensor_read_ctx_create: %s\n", ipmi_ctx_errormsg(smfd_ipmi));

	SMFD_NOTICE("Setting BMC fan management mode to full (manual)\n");
	smfd_set_fan_mode(SMFD_SUPERMICRO_FAN_MODE_FULL);

	SMFD_NOTICE("Setting CPU fan to 100%%\n");
	smfd_set_fan_percent(SMFD_FAN_ZONE_CPU, 100);

	SMFD_NOTICE("Setting system fan to 100%%\n");
	smfd_set_fan_percent(SMFD_FAN_ZONE_SYS, 100);

	SMFD_DEBUG("smfd_ipmi_init finished\n");
}

/* Close/destroy FreeIPMI contexts (smfd_ipmi and smfd_read) & free smfd_ipmi_fans */
static void smfd_ipmi_fini(void)
{
	unsigned i;

	ipmi_sensor_read_ctx_destroy(smfd_read);

	if (ipmi_ctx_close(smfd_ipmi) < 0)
		SMFD_ERR("ipmi_ctx_close: %s\n", ipmi_ctx_errormsg(smfd_ipmi));

	ipmi_ctx_destroy(smfd_ipmi);

	for (i = 0; i < smfd_ipmi_fan_count; ++i)
		free(smfd_ipmi_fans[i].name);

	free(smfd_ipmi_fans);
}

/* Read the current RPM of all IPMI fans */
static void smfd_ipmi_fan_read(void)
{
	uint16_t bitmask;
	double *reading;
	unsigned i;
	int rc;

	for (i = 0; i < smfd_ipmi_fan_count; ++i) {

		rc = ipmi_sensor_read(smfd_read, smfd_ipmi_fans[i].record,
				      smfd_ipmi_fans[i].record_len, 0, NULL, &reading, &bitmask);
		if (rc <= 0) {
			SMFD_FATAL("ipmi_sensor_read: %s\n",
				   ipmi_sensor_read_ctx_errormsg(smfd_read));
		}

		if (*reading < 0 || *reading > UINT_MAX) {
			SMFD_FATAL("%s fan (%g RPM) out of range\n",
				   smfd_ipmi_fans[i].name, *reading);
		}

		smfd_ipmi_fans[i].rpm = *reading;
		free(reading);
	}
}


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	Process temperatures/thresholds to determine fan percentages
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Process 1 temperature against a set of thresholds */
static void smfd_process_temp(const int temp, struct smfd_temp_threshold *const cfg,
			      const char *const name, struct smfd_process_temp_result *const result)
{
	struct smfd_temp_threshold *t, *max;

	for (max = NULL, t = cfg; t->name != NULL; ++t) {

		if (t->active) {
			if (temp >= t->hysteresis) {
				SMFD_DEBUG("%s temperature (%d) still exceeds %s hysteresis (%d)\n",
					   name, temp, t->name, t->hysteresis);
				/* no need to set t-> active; it already is */
				max = t;
			}
			else {
				SMFD_INFO("%s temperature (%d) "
					  "no longer exceeds %s hysteresis (%d)\n",
					  name, temp, t->name, t->hysteresis);
				t->active = 0;
			}
		}
		else {
			if (temp >= t->threshold) {
				SMFD_INFO("%s temperature (%d) exceeds %s threshold (%d)\n",
					  name, temp, t->name, t->hysteresis);
				t->active = 1;
				max = t;
			}
		}
	}

	if (max != NULL) {
		SMFD_DEBUG("%s temperature (%d) ==> "
			   "%s fan settings (CPU: %" PRIu8 "%%, SYS: %" PRIu8 "%%)\n",
			   name, temp, max->name, max->cpu_fan_percent, max->sys_fan_percent);
		result->cpu_fan_percent = max->cpu_fan_percent;
		result->cpu_threshold = max;
		result->sys_fan_percent = max->sys_fan_percent;
		result->sys_threshold = max;
	}
	else {
		SMFD_DEBUG("%s temperature (%d) ==> "
			   "base fan settings (CPU: %" PRIu8 "%%, SYS: %" PRIu8 "%%)\n",
			   name, temp, smfd_cpu_fan_base, smfd_sys_fan_base);
		result->cpu_fan_percent = smfd_cpu_fan_base;
		result->cpu_threshold = NULL;
		result->sys_fan_percent = smfd_sys_fan_base;
		result->sys_threshold = NULL;
	}
}

/* Process the PCH temperature/thresholds */
static void smfd_process_pch_temp(struct smfd_process_temp_result *const result)
{
	smfd_process_temp(smfd_pch_temp.current, smfd_cfg_pch_temp, "PCH", result);
}

/* Process the highest CPU (coretemp) temperature/thresholds */
static void smfd_process_cpu_temps(struct smfd_process_temp_result *const result)
{
	struct smfd_coretemp *max;
	unsigned i;

	for (max = &smfd_coretemps[0], i = 1; i < smfd_coretemp_count; ++i) {
		if (smfd_coretemps[i].temp.current > max->temp.current)
			max = &smfd_coretemps[i];
	}

	SMFD_DEBUG("Highest CPU temperature is %d (%s)\n", max->temp.current, max->name);

	smfd_process_temp(max->temp.current, smfd_cfg_cpu_temp, "CPU", result);
}

/* Process the highest disk temperature/thresholds */
static void smfd_process_disk_temps(struct smfd_process_temp_result *const result)
{
	struct smfd_disk *max;
	unsigned i;

	for (max = &smfd_disks[0], i = 1; i < smfd_disk_count; ++i) {
		if (smfd_disks[i].temp.current > max->temp.current)
			max = &smfd_disks[i];
	}

	SMFD_DEBUG("Highest disk temperature is %d (%s)\n", max->temp.current, max->name);

	smfd_process_temp(max->temp.current, smfd_cfg_disk_temp, "disk",result);
}

/* Process all temperature readings and set the fan speeds */
static void smfd_process_all_temps(void)
{
	struct smfd_process_temp_result results[3] = {
		{ .name = "PCH" },
		{ .name = "CPU" },
		{ .name = "disk" }
	};

	const struct smfd_process_temp_result *cpu, *sys;
	unsigned i;

	smfd_process_pch_temp(&results[0]);
	smfd_process_cpu_temps(&results[1]);
	smfd_process_disk_temps(&results[2]);

	for (cpu = sys= &results[0], i = 1; i < 3; ++i) {
		if (results[i].cpu_fan_percent > cpu->cpu_fan_percent)
			cpu = &results[i];
		if (results[i].sys_fan_percent > sys->sys_fan_percent)
			sys = &results[i];
	}

	SMFD_DEBUG("%s temperature ==> CPU fan @ %" PRIu8 "%%\n", cpu->name, cpu->cpu_fan_percent);
	SMFD_DEBUG("%s temperature ==> SYS fan @ %" PRIu8 "%%\n", sys->name, sys->sys_fan_percent);

	if (cpu->cpu_fan_percent != smfd_cpu_fan_percent) {

		if (cpu->cpu_threshold == NULL) {
			SMFD_NOTICE("Setting CPU fan to %" PRIu8 "%%\n", cpu->cpu_fan_percent);
		}
		else {
			SMFD_NOTICE("Setting CPU fan to %" PRIu8 "%% (%s %s threshold)\n",
				    cpu->cpu_fan_percent, cpu->name, cpu->cpu_threshold->name);
		}

		smfd_set_fan_percent(SMFD_FAN_ZONE_CPU, cpu->cpu_fan_percent);
		smfd_cpu_fan_percent = cpu->cpu_fan_percent;
	}

	if (sys->sys_fan_percent != smfd_sys_fan_percent) {

		if (sys->sys_threshold == NULL) {
			SMFD_NOTICE("Setting system fan to %" PRIu8 "%%\n", sys->sys_fan_percent);
		}
		else {
			SMFD_NOTICE("Setting system fan to %" PRIu8 "%% (%s %s threshold)\n",
				    sys->sys_fan_percent, sys->name, sys->sys_threshold->name);
		}

		smfd_set_fan_percent(SMFD_FAN_ZONE_SYS, sys->sys_fan_percent);
		smfd_sys_fan_percent = sys->sys_fan_percent;
	}
}


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	Configuration file
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Print/log config settings in smfd_cfg_cpu_temp, smfd_cfg_pch_temp or smfd_cfg_disk_temp */
static void smfd_dump_threshold_config(const char *const restrict name,
				       const struct smfd_temp_threshold *thresh)
{
	unsigned int i;

	SMFD_DEBUG("  %s:\n", name);

	for (i = 0; thresh->name != NULL; ++i, ++thresh) {
		SMFD_DEBUG("    [%u]:\n", i);
		SMFD_DEBUG("      .name: %s\n", thresh->name);
		SMFD_DEBUG("      .threshold: %d\n", thresh->threshold);
		SMFD_DEBUG("      .hysteresis: %d\n", thresh->hysteresis);
		SMFD_DEBUG("      .cpu_fan_percent: %" PRIu8 "\n", thresh->cpu_fan_percent);
		SMFD_DEBUG("      .sys_fan_percent: %" PRIu8 "\n", thresh->sys_fan_percent);
	}
}

/* Print/log all configuration settings */
static void smfd_dump_config(void)
{
	unsigned int i;

	if (!smfd_debug)
		return;

	SMFD_DEBUG("  smfd_sdr_cache: %s\n", smfd_sdr_cache);
	SMFD_DEBUG("  smfd_log_interval: %u\n", smfd_log_interval);
	SMFD_DEBUG("  smfd_cpu_fan_base: %" PRIu8 "\n", smfd_cpu_fan_base);
	SMFD_DEBUG("  smfd_sys_fan_base: %" PRIu8 "\n", smfd_sys_fan_base);

	smfd_dump_threshold_config("smfd_cfg_cpu_temp", smfd_cfg_cpu_temp);
	smfd_dump_threshold_config("smfd_cfg_pch_temp", smfd_cfg_pch_temp);
	smfd_dump_threshold_config("smfd_cfg_disk_temp", smfd_cfg_disk_temp);

	SMFD_DEBUG("  smfd_ipmi_fans:\n");

	for (i = 0; i < smfd_ipmi_fan_count; ++i) {
		SMFD_DEBUG("    [%u]:\n", i);
		SMFD_DEBUG("      .record_id: %" PRIu16 "\n", smfd_ipmi_fans[i].record_id);
		SMFD_DEBUG("      .name: %s\n", smfd_ipmi_fans[i].name);
	}

	SMFD_DEBUG("  smfd_disks:\n");

	for (i = 0; i < smfd_disk_count; ++i) {
		SMFD_DEBUG("    [%u]:\n", i);
		SMFD_DEBUG("      .name: %s\n", smfd_disks[i].name);
	}

	if (smfd_config_test)
		exit(EXIT_SUCCESS);
}

/* Fatal error if the node is not of the expected type */
static void smfd_check_node_type(const yaml_node_t *const node, const char *const restrict name,
				 const yaml_node_type_t type, const char *const restrict type_name)
{
	if (node->type != type)
		SMFD_CFG_FATAL("value of %s is not a %s\n", node, name, type_name);
}

/* Fatal error if the node is not a YAML_SCALAR_NODE */
static void smfd_check_scalar(const yaml_node_t *const node, const char *const restrict name)
{
	smfd_check_node_type(node, name, YAML_SCALAR_NODE, "scalar");
}

/* Fatal error if the node is not a YAML_SEQUENCE_NODE */
static void smfd_check_sequence(const yaml_node_t *const node, const char *const restrict name)
{
	smfd_check_node_type(node, name, YAML_SEQUENCE_NODE, "sequence");
}

/* Fatal error if the node is not a YAML_MAPPING_NODE */
static void smfd_check_mapping(const yaml_node_t *node, const char *const restrict name)
{
	smfd_check_node_type(node, name, YAML_MAPPING_NODE, "mapping");
}

/* Parse an integer from a scalar node */
static int smfd_parse_int(const yaml_node_t *const node, const char *const restrict name)
{
	long value;
	char *end;

	smfd_check_scalar(node, name);

	if (*node->data.scalar.value == 0 || isspace(*node->data.scalar.value)) {
		SMFD_CFG_FATAL("value of %s (%s) is not a valid integer\n",
			       node, name, node->data.scalar.value);
	}

	errno = 0;
	value = strtol((char *)node->data.scalar.value, &end, 0);

	if (errno != 0 || *end != 0 || value < INT_MIN || value > INT_MAX) {
		SMFD_CFG_FATAL("value of %s (%s) is not a valid integer\n",
			       node, name, node->data.scalar.value);
	}

	return (int)value;
}

/* Parse (allocate & copy) a string from a scalar node */
static char *smfd_parse_string(const yaml_node_t *const node, const char *const restrict name)
{
	char *value;

	smfd_check_scalar(node, name);

	if ((value = malloc(node->data.scalar.length + 1)) == NULL)
		SMFD_ABORT("malloc: %m\n");

	memcpy(value, node->data.scalar.value, node->data.scalar.length + 1);

	return value;
}

static void smfd_parse_sdr_cache(const yaml_node_t *const node,
				 yaml_document_t *const doc __attribute__((unused)),
				 const char *const restrict name,
				 void *const restrict data __attribute__((unused)))
{
	smfd_sdr_cache = smfd_parse_string(node, name);
}

/* Parse a fan speed (percentage) from a scalar node */
static void smfd_parse_fan_speed(const yaml_node_t *const node,
				 yaml_document_t *const doc __attribute__((unused)),
				 const char *const restrict name, void *const restrict data)
{
	uint8_t *const speed = data;
	int value;

	value = smfd_parse_int(node, name);

	if (value < 0 || value > 100)
		SMFD_CFG_FATAL("%s (%d%%) is not a valid fan speed\n", node, name, value);

	if (value < 25) {
		SMFD_WARNING("Fan speeds below 25%% may cause problems (%s = %d%%)\n",
			     name, value);
	}

	*speed = (uint8_t)value;
}

/* Parse a logging interval (seconds) from a scalar node */
static void smfd_parse_log_interval(const yaml_node_t *const node,
				    yaml_document_t *const doc __attribute__((unused)),
				    const char *const restrict name, void *const restrict data)
{
	unsigned int *const interval = data;
	int value;

	value = smfd_parse_int(node, name);

	if (value < 0)
		SMFD_CFG_FATAL("%s (%d) is not a valid logging interval\n", node, name, value);

	if (value != 0 && value < 30)
		SMFD_WARNING("%s (%d) is less than 30 second sampling interval\n", name, value);

	if (value != 0 && value < 600)
		SMFD_WARNING("%s (%d seconds) may generate excessive log entries\n", name, value);

	if (value > 30000000) /* about 1 year */ {
		SMFD_WARNING("Set %s to 0 to disable periodic logging (%s = %d)\n",
			     name, name, value);
	}

	*interval = (unsigned int)value;
}

/* Parse a temperature from a scalar node */
static int smfd_parse_temp(const yaml_node_t *const node, const char *const restrict name)
{
	int value;

	value = smfd_parse_int(node, name);

	if (value < -273 || value > 999)
		SMFD_CFG_FATAL("%s (%d) is not a valid temperature\n", node, name, value);

	if (value < 25 || value > 80) {
		SMFD_WARNING("Temperatures outside 25°C - 80°C are probably not useful (%s = %d)\n",
			     name, value);
	}

	return value;
}

/* Parse an IPMI SDR record ID from a scalar node */
static uint16_t smfd_parse_record_id(const yaml_node_t *const node)
{
	int value;

	value = smfd_parse_int(node, "record_id");

	if (value < 0 || value >= 0xffff)
		SMFD_CFG_FATAL("record_id (%d) is not a valid IPMI SDR ID\n", node, value);

	return (uint16_t)value;
}

/* Fatal error due to missing field (key) in a mapping node */
__attribute__((noreturn))
static void smfd_missing_field(const yaml_node_t *const node, const char *const restrict seq_name,
			       const char *const restrict field_name)
{
	SMFD_CFG_FATAL("%s not set in %s element\n", node, field_name, seq_name);
}

/* Parse smfd_ipmi_fans and smfd_ipmi_fan_count from a sequence node */
static void smfd_parse_ipmi_fans(const yaml_node_t *const node, yaml_document_t *const doc,
				 const char *const restrict name,
				 void *const restrict data __attribute__((unused)))
{
	const yaml_node_t *map, *key, *value;
	const yaml_node_item_t *item;
	const yaml_node_pair_t *kv;
	struct smfd_ipmi_fan *fans;
	ptrdiff_t len;
	int i;

	smfd_check_sequence(node, name);

	len = node->data.sequence.items.top - node->data.sequence.items.start;
	assert(len > 0);

	if ((fans = malloc(len * sizeof *fans)) == NULL)
		SMFD_ABORT("malloc: %m\n");

	for (i = 0, item = node->data.sequence.items.start ; i < len; ++i, ++item) {

		map = yaml_document_get_node(doc, *item);
		smfd_check_mapping(map, name);
		fans[i].name = NULL;
		fans[i].record_id = 0xffff;

		for (kv = map->data.mapping.pairs.start; kv < map->data.mapping.pairs.top; ++kv) {

			key = yaml_document_get_node(doc, kv->key);
			if (key->type != YAML_SCALAR_NODE)
				SMFD_CFG_FATAL("mapping key is not a scalar\n", key);

			value = yaml_document_get_node(doc, kv->value);

			if (strcmp((char *)key->data.scalar.value, "name") == 0) {
				fans[i].name = smfd_parse_string(value, "name");
			}
			else if (strcmp((char *)key->data.scalar.value, "record_id") == 0) {
				fans[i].record_id = smfd_parse_record_id(value);
			}
			else {
				SMFD_CFG_FATAL("unknown key (%s) in ipmi_fans\n",
					       key, key->data.scalar.value);
			}
		}

		if (fans[i].name == NULL)
			smfd_missing_field(map, "ipmi_fans", "name");
		if (fans[i].record_id == 0xffff)
			smfd_missing_field(map, "ipmi_fans", "record_id");
	}

	smfd_ipmi_fans = fans;
	smfd_ipmi_fan_count = len;
}

/*
 * Parse a trigger in smfd_cfg_cpu_temp, smfd_cfg_pch_temp or smfd_cfg_disk_temp from a mapping node
 */
static void smfd_parse_trigger(const yaml_node_t *const node, yaml_document_t *const doc,
			       const char *const restrict name,
			       struct smfd_temp_threshold *const trigger)
{
	static const struct smfd_temp_threshold init = {
		.name			= NULL,
		.threshold		= INT_MIN,
		.hysteresis		= INT_MIN,
		.cpu_fan_percent	= 255,
		.sys_fan_percent	= 255,
		.active			= 1	/* not a flag value; all triggers start active */
	};

	const yaml_node_t *key, *value;
	const yaml_node_pair_t *pair;

	smfd_check_mapping(node, name);
	memcpy(trigger, &init, sizeof *trigger);

	for (pair = node->data.mapping.pairs.start; pair < node->data.mapping.pairs.top; ++pair) {

		key = yaml_document_get_node(doc, pair->key);
		if (key->type != YAML_SCALAR_NODE)
			SMFD_CFG_FATAL("mapping key is not a scalar\n", key);

		value = yaml_document_get_node(doc, pair->value);

		if (strcmp((char *)key->data.scalar.value, "name") == 0) {
			trigger->name = smfd_parse_string(value, "name");
		}
		else if (strcmp((char *)key->data.scalar.value, "threshold") == 0) {
			trigger->threshold = smfd_parse_temp(value, "threshold");
		}
		else if (strcmp((char *)key->data.scalar.value, "hysteresis") == 0) {
			trigger->hysteresis = smfd_parse_temp(value, "hysteresis");
		}
		else if (strcmp((char *)key->data.scalar.value, "cpu_fan_speed") == 0) {
			smfd_parse_fan_speed(value, doc, "cpu_fan_speed",
					     &trigger->cpu_fan_percent);
		}
		else if (strcmp((char *)key->data.scalar.value, "sys_fan_speed") == 0) {
			smfd_parse_fan_speed(value, doc, "sys_fan_speed",
					     &trigger->sys_fan_percent);
		}
		else {
			SMFD_CFG_FATAL("unknown key (%s) in %s\n",
				       key, key->data.scalar.value, name);
		}
	}

	if (trigger->name == NULL)
		smfd_missing_field(node, name, "name");
	if (trigger->threshold == INT_MIN)
		smfd_missing_field(node, name, "threshold");
	if (trigger->hysteresis == INT_MIN)
		smfd_missing_field(node, name, "hysteresis");

	if (trigger->cpu_fan_percent == 255 && trigger->sys_fan_percent == 255)
		SMFD_CFG_FATAL("no cpu_fan_speed or sys_fan_speed in %s element\n", node, name);

	trigger->cpu_fan_percent = (trigger->cpu_fan_percent == 255) ? 0 : trigger->cpu_fan_percent;
	trigger->sys_fan_percent = (trigger->sys_fan_percent == 255) ? 0 : trigger->sys_fan_percent;

	if (trigger->hysteresis >= trigger->threshold) {
		SMFD_CFG_FATAL("hysteresis (%d) >= threshold (%d) in %s element\n",
			       node, trigger->hysteresis, trigger->threshold, name);
	}
}

/* Parse smfd_cfg_cpu_temp, smfd_cfg_pch_temp or smfd_cfg_disk_temp from a sequence node */
static void smfd_parse_triggers(const yaml_node_t *const node, yaml_document_t *const doc,
				const char *const restrict name, void *const restrict data)
{
	struct smfd_temp_threshold **const triggers = data;

	const yaml_node_item_t *item;
	ptrdiff_t len;
	int i;

	smfd_check_sequence(node, name);

	len = node->data.sequence.items.top - node->data.sequence.items.start;
	assert(len > 0);

	if ((*triggers = malloc((len + 1) * sizeof **triggers)) == NULL)
		SMFD_ABORT("malloc: %m\n");

	(*triggers)[len].name = NULL;

	for (i = 0, item = node->data.sequence.items.start ; i < len; ++i, ++item)
		smfd_parse_trigger(yaml_document_get_node(doc, *item), doc, name, &(*triggers)[i]);
}

/* Parse smfd_disks and smfd_disk_count from a sequence node */
static void smfd_parse_smart_disks(const yaml_node_t *const node, yaml_document_t *const doc,
				   const char *const restrict name,
				   void *const restrict data __attribute__((unused)))
{
	const yaml_node_item_t *item;
	struct smfd_disk *disks;
	ptrdiff_t len;
	int i;

	smfd_check_sequence(node, name);

	len = node->data.sequence.items.top - node->data.sequence.items.start;
	assert(len > 0);

	if ((disks = malloc(len * sizeof *disks)) == NULL)
		SMFD_ABORT("malloc: %m\n");

	for (i = 0, item = node->data.sequence.items.start ; i < len; ++i, ++item)
		disks[i].name = smfd_parse_string(yaml_document_get_node(doc, *item), name);

	smfd_disks = disks;
	smfd_disk_count = len;
}

/* Fatal error due to missing key in configuration file */
__attribute__((noreturn))
static void smfd_missing_config(const char *const name)
{
	SMFD_FATAL("Invalid configuration: %s: %s not set\n", smfd_config_file, name);
}

/* Load and parse the configuration file */
static void smfd_load_config(void)
{
	static const struct {
		const char *name;
		void (*parse_fn)(const yaml_node_t *node, yaml_document_t *const doc,
				 const char *restrict name, void *data);
		void *data;
	}
	parse_fns[] = {
		{ "cpu_fan_base",	smfd_parse_fan_speed,		&smfd_cpu_fan_base	},
		{ "sys_fan_base",	smfd_parse_fan_speed,		&smfd_sys_fan_base	},
		{ "log_interval",	smfd_parse_log_interval,	&smfd_log_interval	},
		{ "cpu_temp_triggers",	smfd_parse_triggers,		&smfd_cfg_cpu_temp	},
		{ "pch_temp_triggers",	smfd_parse_triggers,		&smfd_cfg_pch_temp	},
		{ "disk_temp_triggers",	smfd_parse_triggers,		&smfd_cfg_disk_temp	},
		{ "ipmi_fans",		smfd_parse_ipmi_fans,		NULL			},
		{ "smart_disks",	smfd_parse_smart_disks,		NULL			},
		{ "sdr_cache_file",	smfd_parse_sdr_cache,		NULL			},
		{ NULL }
	};

	const yaml_node_t *node, *key;
	const yaml_node_pair_t *pair;
	yaml_parser_t parser;
	yaml_document_t doc;
	unsigned int i;
	FILE *fp;

	if ((fp = fopen(smfd_config_file, "r")) == NULL)
		SMFD_FATAL("%s: %m\n", smfd_config_file);

	if (!yaml_parser_initialize(&parser))
		SMFD_LIBYAML_FATAL(smfd_config_file, &parser);

	yaml_parser_set_input_file(&parser, fp);

	if (!yaml_parser_load(&parser, &doc))
		SMFD_LIBYAML_FATAL(smfd_config_file, &parser);

	yaml_parser_delete(&parser);

	if (fclose(fp) != 0)
		SMFD_FATAL("fclose: %m\n");

	if ((node = yaml_document_get_root_node(&doc)) == NULL)
		SMFD_ABORT("yaml_document_get_root_node: %m\n");

	if (node->type != YAML_MAPPING_NODE)
		SMFD_FATAL("Invalid configuration: %s: not a YAML mapping\n", smfd_config_file);

	for (pair = node->data.mapping.pairs.start; pair < node->data.mapping.pairs.top; ++pair) {

		key = yaml_document_get_node(&doc, pair->key);
		if (key->type != YAML_SCALAR_NODE)
			SMFD_CFG_FATAL("mapping key is not a scalar\n", key);

		for (i = 0; parse_fns[i].name != NULL; ++i) {
			if (strcmp((char *)key->data.scalar.value, parse_fns[i].name) == 0) {
				parse_fns[i].parse_fn(yaml_document_get_node(&doc, pair->value),
						&doc, parse_fns[i].name, parse_fns[i].data);
				break;
			}
		}

		if (parse_fns[i].name == NULL)
			SMFD_CFG_FATAL("unknown key (%s)\n", key, key->data.scalar.value);
	}

	yaml_document_delete(&doc);

	if (smfd_cpu_fan_base == 255)		smfd_missing_config("cpu_fan_base");
	if (smfd_sys_fan_base == 255)		smfd_missing_config("sys_fan_base");
	if (smfd_log_interval == UINT_MAX)	smfd_missing_config("log_interval");
	if (smfd_disks == NULL)			smfd_missing_config("smart_disks");
	if (smfd_ipmi_fans == NULL)		smfd_missing_config("ipmi_fans");
	if (smfd_cfg_cpu_temp == NULL)		smfd_missing_config("cpu_temp_triggers");
	if (smfd_cfg_pch_temp == NULL)		smfd_missing_config("pch_temp_triggers");
	if (smfd_cfg_disk_temp == NULL)		smfd_missing_config("disk_temp_triggers");
}


/***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **
 **
 **	Main loop, signal handling, etc.
 **
 **
 ***************************************************************************************************
 ***************************************************************************************************
 ***************************************************************************************************
 **************************************************************************************************/

/* Signal handler */
static void smfd_signal_handler(const int signal)
{
	switch (signal) {

		case SIGUSR1:	smfd_debug_signal = 1;
				break;

		case SIGUSR2:	smfd_dump_signal = 1;
				break;

		case SIGTERM:
		case SIGINT:	smfd_quit_signal = 1;
	}
}

/* Set up signal handlers */
static void smfd_signal_init(void)
{
	static const struct sigaction act = { .sa_handler = smfd_signal_handler };
	static const int signals[] = { SIGUSR1, SIGUSR2, SIGTERM, SIGINT, 0 };

	const int *s;

	for (s = signals; *s != 0; ++s) {
		if (sigaction(*s, &act, NULL) != 0)
			SMFD_FATAL("sigaction: %m\n");
	}
}

/* Close files, free memory, etc. */
static void smfd_cleanup(void)
{
	struct smfd_temp_threshold *trigger;

	smfd_disk_fini();
	smfd_ipmi_fini();
	smfd_pch_temp_fini();
	smfd_coretemp_fini();

	for (trigger = smfd_cfg_cpu_temp; trigger->name != NULL; ++trigger)
		free(trigger->name);

	for (trigger = smfd_cfg_pch_temp; trigger->name != NULL; ++trigger)
		free(trigger->name);

	for (trigger = smfd_cfg_disk_temp; trigger->name != NULL; ++trigger)
		free(trigger->name);

	free(smfd_cfg_cpu_temp);
	free(smfd_cfg_pch_temp);
	free(smfd_cfg_disk_temp);
}

/* Process smfd_debug_signal and smfd_dump_signal */
static void smfd_check_signals(void)
{
	static const char bool_names[2][sizeof "OFF"] = { "OFF", "ON" };

	if (smfd_debug_signal) {
		SMFD_NOTICE("Got SIGUSR1; switching debugging from %s to %s\n",
			    bool_names[smfd_debug], bool_names[!smfd_debug]);
		smfd_debug = !smfd_debug;
		smfd_debug_signal = 0;
	}

	if (smfd_dump_signal) {
		SMFD_NOTICE("Got SIGUSR2; logging some stuff\n");
		smfd_log_info();
		smfd_dump_signal = 0;
	}
}

static void smfd_log_init(void)
{
	if (smfd_log_interval == 0)
		return;

	smfd_log_start = time(NULL);
	smfd_next_log = smfd_log_start + smfd_log_interval;
}

static void smfd_log_check(void)
{
	time_t now;

	if (smfd_log_interval == 0)
		return;

	now = time(NULL);

	if (now >= smfd_next_log) {
		smfd_log_info();
		smfd_log_start = now;
		smfd_next_log = smfd_log_start + smfd_log_interval;
	}
}

#include <mcheck.h>

int main(const int argc, char **const argv)
{
	mtrace();

	smfd_parse_args(argc, argv);
	smfd_load_config();
	smfd_dump_config();

	smfd_signal_init();
	smfd_coretemp_init();
	smfd_pch_temp_init();
	smfd_ipmi_init();
	smfd_disk_init();
	smfd_log_init();


	while (!smfd_quit_signal) {

		smfd_check_signals();

		smfd_coretemp_read();
		smfd_pch_temp_read();
		smfd_disk_read();

		smfd_process_all_temps();

		smfd_log_check();

		sleep(30);
	};

	SMFD_NOTICE("Got shutdown signal\n");

	smfd_cleanup();

	return 0;
}
