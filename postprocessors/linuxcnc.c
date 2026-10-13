#include "gcode_planner.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

int sg_write_linuxcnc(FILE *output, const sg_machine *machine,
                      const sg_toolpath *path, const sg_plan *plan,
                      sg_diagnostic *diagnostic) {
    if (machine->count != 5)
        return sg_fail(diagnostic, SG_DIALECT, 0, "LinuxCNC head5 output requires five direct XYZBC coordinates; twelve joints need a separate controller adapter");
    sg_plan *rounded = malloc(sizeof *rounded);
    if (!rounded) return sg_fail(diagnostic, SG_CAPACITY, 0, "output validation allocation failed");
    *rounded = *plan;
    /* Validate the exact printed coordinates and inverse-time words, rather
       than only the more precise internal trajectory. */
    for (int point = 0; point < plan->count; ++point) {
        for (int axis = 0; axis < machine->count; ++axis) {
            double unit = machine->axes[axis].rotary ? 180/SG_PI : 1;
            double printed = round(plan->configuration[point].joint[axis]*unit*1e10)/1e10;
            rounded->configuration[point].joint[axis] = printed/unit;
        }
        if (point) {
            double inverse_time = floor((60/plan->seconds[point])*1e10)/1e10;
            if (inverse_time <= 0) { free(rounded); return sg_fail(diagnostic,SG_INPUT,point,"inverse-time word underflow"); }
            rounded->seconds[point] = 60/inverse_time;
        }
    }
    int result = sg_validate_plan(machine,path,rounded,diagnostic);
    if (result) { free(rounded); return result; }
    fprintf(output,"(SIMULATION PROTOTYPE - controller and machine validation required)\n");
    fprintf(output,"(XYZ are spindle-nose coordinates; BC right-handed head rotations)\n");
    fprintf(output,"(Requires trivkins coordinates=XYZBC, zero G54/G52/G92 and stated initial joints)\n");
    fprintf(output,"(Tool already installed, modeled length %.10f mm; G49 prevents double compensation)\n",machine->tool_length);
    fprintf(output,"G21 G90 G17 G40 G49 G80 G54 G61\nG93\nS%.10f M3\n",machine->spindle_rpm);
    fprintf(output,"(INITIAL X%.10f Y%.10f Z%.10f B%.10f C%.10f - no positioning move emitted)\n",
        rounded->configuration[0].joint[0],rounded->configuration[0].joint[1],rounded->configuration[0].joint[2],
        rounded->configuration[0].joint[3]*180/SG_PI,rounded->configuration[0].joint[4]*180/SG_PI);
    for (int point = 1; point < rounded->count; ++point) {
        const double *joint = rounded->configuration[point].joint;
        fprintf(output,"(source %d %s %s %s)\n",point,path->points[point].object,path->points[point].operation,path->points[point].cutting?"cut":"travel");
        fprintf(output,"N%d G1 X%.10f Y%.10f Z%.10f B%.10f C%.10f F%.10f\n",point*10,
            joint[0],joint[1],joint[2],joint[3]*180/SG_PI,joint[4]*180/SG_PI,60/rounded->seconds[point]);
    }
    fprintf(output,"M5\nM9\nG94 F100\nM2\n");
    free(rounded);
    if (ferror(output)) return sg_fail(diagnostic,SG_IO,0,"G-code write failed");
    return SG_OK;
}

int sg_write_trace(FILE *output, const sg_machine *machine,
                   const sg_toolpath *path, const sg_plan *plan) {
    fprintf(output,"point\tobject\toperation\tmode\tseconds\tbranch\tminimum_sigma\tclearance_mm");
    for (int axis = 0; axis < machine->count; ++axis) fprintf(output,"\t%c_%s",machine->axes[axis].name,machine->axes[axis].rotary?"rad":"mm");
    fprintf(output,"\n");
    for (int point = 0; point < plan->count; ++point) {
        const double *joint = plan->configuration[point].joint;
        fprintf(output,"%d\t%s\t%s\t%s\t%.12g\t%d\t%.12g\t%.12g",point,path->points[point].object,path->points[point].operation,
            path->points[point].cutting?"cut":"travel",plan->seconds[point],plan->configuration[point].branch,
            sg_analyze(machine,joint,0,0).minimum_singular_value,sg_clearance(machine,joint));
        for (int axis = 0; axis < machine->count; ++axis) fprintf(output,"\t%.12g",joint[axis]);
        fprintf(output,"\n");
    }
    return ferror(output) ? SG_IO : SG_OK;
}
