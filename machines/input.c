#include "gcode_planner.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

int sg_fail(sg_diagnostic *diagnostic, int code, int point, const char *message) {
    if (diagnostic) {
        diagnostic->code = code;
        diagnostic->point = point;
        snprintf(diagnostic->message, sizeof diagnostic->message, "%s", message);
    }
    return code;
}

void sg_default_machine(sg_machine *machine, int count) {
    memset(machine, 0, sizeof *machine);
    machine->count = count;
    machine->tool_length = 80;
    machine->tool_radius = 1;
    machine->head_radius = 5;
    machine->housing_offset = 10;
    machine->position_tolerance = 0.03;
    machine->direction_tolerance = 0.002;
    machine->spindle_rpm = 1200;
    const char *names = "XYZBCUVWPQAE";
    for (int axis = 0; axis < SG_AXES; ++axis) {
        int rotary = axis == 3 || axis == 4 || axis >= 10;
        machine->axes[axis] = (sg_axis){names[axis], rotary,
            rotary ? -3 * SG_PI : -250, rotary ? 3 * SG_PI : 250,
            rotary ? 1 : 100, rotary ? 2 : 200, rotary ? 10 : 1000, 0};
    }
    machine->axes[3].lower = -2;
    machine->axes[3].upper = 2;
    machine->axes[2].initial = 130;
    for (int axis = 10; axis < 12; ++axis) {
        machine->axes[axis].lower = -SG_PI / 3;
        machine->axes[axis].upper = SG_PI / 3;
    }
}

/* Whitespace-delimited UTF-8 text; comments start with #. No executable input.
   Every line is bounded and every numeric token must be finite. */
static int tokens(FILE *input, char buffer[1024], char *words[20], int *line) {
    while (fgets(buffer, 1024, input)) {
        ++*line;
        if (!strchr(buffer, '\n') && !feof(input)) return -1;
        char *comment = strchr(buffer, '#');
        if (comment) *comment = '\0';
        int count = 0;
        char *word = strtok(buffer, " \t\r\n");
        while (word) {
            if (count == 20) return -1;
            words[count++] = word;
            word = strtok(NULL, " \t\r\n");
        }
        if (count) return count;
    }
    return ferror(input) ? -1 : 0;
}
static int number(const char *word, double *value) {
    char *end;
    *value = strtod(word, &end);
    return end != word && *end == '\0' && isfinite(*value);
}
static int identifier(const char *word) {
    if (!*word || strlen(word) >= 64) return 0;
    for (; *word; ++word)
        if (!(isalnum((unsigned char)*word) || *word == '-' || *word == '_' || *word == '/')) return 0;
    return 1;
}

int sg_read_machine(FILE *input, sg_machine *machine, sg_diagnostic *diagnostic) {
    char buffer[1024], *words[20];
    int line = 0, count, axis_seen = 0, model_seen = 0, tool_seen = 0;
    int tolerance_seen = 0, spindle_seen = 0, fitting_seen = 0;
    sg_default_machine(machine, 5);
    while ((count = tokens(input, buffer, words, &line)) > 0) {
        if (!strcmp(words[0], "machine") && count == 3 && !model_seen && !strcmp(words[2], "mm-radians-v1")) {
            if (!strcmp(words[1], "head5")) sg_default_machine(machine, 5);
            else if (!strcmp(words[1], "hybrid12")) sg_default_machine(machine, 12);
            else return sg_fail(diagnostic, SG_INPUT, line, "unknown kinematic model");
            model_seen = 1;
            continue;
        }
        if (!model_seen) return sg_fail(diagnostic, SG_INPUT, line, "machine header must precede all geometry");
        double values[16];
        if (!strcmp(words[0], "axis") && count == 9 && strlen(words[1]) == 1) {
            int axis = 0;
            while (axis < machine->count && machine->axes[axis].name != words[1][0]) ++axis;
            if (axis == machine->count || axis_seen & (1 << axis))
                return sg_fail(diagnostic, SG_INPUT, line, "unknown or duplicate axis");
            for (int entry = 3; entry < 9; ++entry)
                if (!number(words[entry], &values[entry-3])) return sg_fail(diagnostic, SG_INPUT, line, "invalid axis number");
            if (strcmp(words[2], machine->axes[axis].rotary ? "rotary" : "linear") ||
                values[0] >= values[1] || values[2] <= 0 || values[3] <= 0 || values[4] <= 0 ||
                values[5] < values[0] || values[5] > values[1] || fabs(values[0]) > 1e6 || fabs(values[1]) > 1e6 ||
                (machine->axes[axis].rotary && values[1] - values[0] > 6 * SG_PI + 1e-8))
                return sg_fail(diagnostic, SG_INPUT, line, "invalid axis units, limits, dynamics or initial state");
            machine->axes[axis] = (sg_axis){words[1][0], machine->axes[axis].rotary,
                values[0], values[1], values[2], values[3], values[4], values[5]};
            axis_seen |= 1 << axis;
        } else if (!strcmp(words[0], "tool") && count == 5 && !tool_seen) {
            for (int entry = 1; entry < 5; ++entry)
                if (!number(words[entry], &values[entry-1]) || values[entry-1] < 0 || values[entry-1] > 1e4)
                    return sg_fail(diagnostic, SG_INPUT, line, "invalid tool geometry");
            if (values[0] <= 0 || values[1] <= 0 || values[2] <= 0)
                return sg_fail(diagnostic, SG_INPUT, line, "tool dimensions must be positive");
            machine->tool_length = values[0]; machine->tool_radius = values[1];
            machine->head_radius = values[2]; machine->housing_offset = values[3]; tool_seen = 1;
        } else if (!strcmp(words[0], "tolerance") && count == 3 && !tolerance_seen) {
            if (!number(words[1], &values[0]) || !number(words[2], &values[1]) ||
                values[0] <= 0 || values[1] <= 0 || values[1] > 0.1 || values[0] > 10)
                return sg_fail(diagnostic, SG_INPUT, line, "invalid geometric tolerances");
            machine->position_tolerance = values[0]; machine->direction_tolerance = values[1]; tolerance_seen = 1;
        } else if (!strcmp(words[0], "spindle") && count == 2 && !spindle_seen) {
            if (!number(words[1], &machine->spindle_rpm) || machine->spindle_rpm <= 0 || machine->spindle_rpm > 100000)
                return sg_fail(diagnostic, SG_INPUT, line, "invalid spindle speed");
            spindle_seen = 1;
        } else if (!strcmp(words[0], "sphere") && count == 5 && machine->obstacle_count < SG_OBSTACLES) {
            for (int entry = 1; entry < 5; ++entry)
                if (!number(words[entry], &values[entry-1]) || fabs(values[entry-1]) > 1e6)
                    return sg_fail(diagnostic, SG_INPUT, line, "invalid fixture sphere");
            if (values[3] <= 0) return sg_fail(diagnostic, SG_INPUT, line, "fixture radius must be positive");
            machine->obstacles[machine->obstacle_count++] = (sg_sphere){{values[0], values[1], values[2]}, values[3]};
        } else if (!strcmp(words[0], "fitting") && count == 7 && !fitting_seen) {
            for (int entry = 1; entry < 7; ++entry)
                if (!number(words[entry], &values[entry-1]) || fabs(values[entry-1]) > 1e4)
                    return sg_fail(diagnostic, SG_INPUT, line, "invalid fitting dimensions");
            for (int entry = 0; entry < 5; ++entry)
                if (values[entry] <= 0) return sg_fail(diagnostic, SG_INPUT, line, "fitting lengths must be positive");
            machine->fitting = 1; fitting_seen = 1;
            machine->stock_half_size = (sg_vector){values[0], values[1], values[2]};
            machine->main_bore_radius = values[3]; machine->branch_bore_radius = values[4]; machine->branch_angle = values[5];
        } else return sg_fail(diagnostic, SG_INPUT, line, "unknown, repeated or malformed machine record");
    }
    if (count < 0 || !model_seen || !tool_seen || !tolerance_seen || !spindle_seen || axis_seen != (1 << machine->count) - 1)
        return sg_fail(diagnostic, SG_INPUT, line, "machine description is incomplete or overlong");
    return SG_OK;
}

int sg_read_toolpath(FILE *input, sg_toolpath *path, sg_diagnostic *diagnostic) {
    char buffer[1024], *words[20];
    int line = 0, count, header = 0;
    path->count = 0;
    while ((count = tokens(input, buffer, words, &line)) > 0) {
        if (!header) {
            if (count != 2 || strcmp(words[0], "toolpath") || strcmp(words[1], "mm-unit-vector-v1"))
                return sg_fail(diagnostic, SG_INPUT, line, "expected toolpath mm-unit-vector-v1");
            header = 1; continue;
        }
        if (count != 10 || !identifier(words[0]) || !identifier(words[1]) || path->count == SG_POINTS)
            return sg_fail(diagnostic, SG_INPUT, line, "malformed waypoint or toolpath capacity exceeded");
        double values[7];
        for (int entry = 3; entry < 10; ++entry)
            if (!number(words[entry], &values[entry-3]) || fabs(values[entry-3]) > 1e6)
                return sg_fail(diagnostic, SG_INPUT, line, "waypoint number is nonfinite or out of range");
        if ((strcmp(words[2], "cut") && strcmp(words[2], "travel")) || values[0] <= 0 || values[0] > 1e4)
            return sg_fail(diagnostic, SG_INPUT, line, "invalid process mode or duration");
        sg_waypoint *waypoint = &path->points[path->count++];
        memset(waypoint, 0, sizeof *waypoint);
        snprintf(waypoint->object, sizeof waypoint->object, "%s", words[0]);
        snprintf(waypoint->operation, sizeof waypoint->operation, "%s", words[1]);
        waypoint->cutting = !strcmp(words[2], "cut"); waypoint->seconds = values[0];
        waypoint->pose.position = (sg_vector){values[1], values[2], values[3]};
        waypoint->pose.direction = (sg_vector){values[4], values[5], values[6]};
        if (fabs(sg_length(waypoint->pose.direction) - 1) > 1e-8)
            return sg_fail(diagnostic, SG_INPUT, line, "tool direction must already be a unit vector");
    }
    if (count < 0 || !header || path->count < 2)
        return sg_fail(diagnostic, SG_INPUT, line, "toolpath must contain at least two bounded waypoints");
    return SG_OK;
}
