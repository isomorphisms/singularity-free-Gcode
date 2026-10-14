#define _POSIX_C_SOURCE 200809L
#include "gcode_planner.h"
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

int main(int argument_count, char **arguments) {
    if (argument_count != 5 || (strcmp(arguments[1],"linuxcnc") && strcmp(arguments[1],"trace"))) {
        fprintf(stderr,"usage: gcode-plan linuxcnc|trace MACHINE TOOLPATH OUTPUT\n"); return SG_INPUT;
    }
    if (!strcmp(arguments[4],arguments[2]) || !strcmp(arguments[4],arguments[3])) {
        fprintf(stderr,"output must differ from input files\n"); return SG_INPUT;
    }
    FILE *machine_file = fopen(arguments[2],"r"), *path_file = fopen(arguments[3],"r");
    sg_machine machine; sg_diagnostic diagnostic = {0};
    sg_toolpath *path = malloc(sizeof *path); sg_plan *plan = malloc(sizeof *plan);
    if (!machine_file || !path_file || !path || !plan) {
        fprintf(stderr,"cannot open inputs or allocate planner\n");
        if (machine_file) fclose(machine_file);
        if (path_file) fclose(path_file);
        free(path); free(plan); return SG_IO;
    }
    struct stat machine_stat, path_stat, destination_stat;
    if (!fstat(fileno(machine_file),&machine_stat) && !fstat(fileno(path_file),&path_stat) &&
        !stat(arguments[4],&destination_stat) &&
        ((destination_stat.st_dev==machine_stat.st_dev && destination_stat.st_ino==machine_stat.st_ino) ||
         (destination_stat.st_dev==path_stat.st_dev && destination_stat.st_ino==path_stat.st_ino))) {
        fprintf(stderr,"output aliases an input file\n"); fclose(machine_file); fclose(path_file);
        free(path); free(plan); return SG_INPUT;
    }
    int result = sg_read_machine(machine_file,&machine,&diagnostic);
    if (!result) result = sg_read_toolpath(path_file,path,&diagnostic);
    fclose(machine_file); fclose(path_file);
    if (!result && !strcmp(arguments[1],"linuxcnc") && machine.count != 5)
        result = sg_fail(&diagnostic,SG_DIALECT,0,"twelve-joint controller mapping is unspecified; use trace for simulation");
    if (!result) result = sg_plan_path(&machine,path,plan,&diagnostic);
    /* Complete planning and emission in a temporary stream. An infeasible
       request never truncates or partially writes an output program. */
    FILE *temporary = NULL;
    if (!result) {
        temporary = tmpfile();
        if (!temporary) result = sg_fail(&diagnostic,SG_IO,0,"cannot create temporary output stream");
    }
    if (!result) result = !strcmp(arguments[1],"linuxcnc") ? sg_write_linuxcnc(temporary,&machine,path,plan,&diagnostic) : sg_write_trace(temporary,&machine,path,plan);
    if (!result && fflush(temporary)) result = sg_fail(&diagnostic,SG_IO,0,"temporary output flush failed");
    if (!result) {
        rewind(temporary);
        size_t filename_length = strlen(arguments[4]);
        char *staging_name = malloc(filename_length+16);
        if (staging_name) snprintf(staging_name,filename_length+16,"%s.tmp.XXXXXX",arguments[4]);
        int descriptor = staging_name ? mkstemp(staging_name) : -1;
        FILE *output = descriptor>=0 ? fdopen(descriptor,"w") : NULL;
        if (!output) {
            if (descriptor>=0) close(descriptor);
            result = sg_fail(&diagnostic,SG_IO,0,"cannot create staged output file");
        }
        else {
            char buffer[8192]; size_t count;
            while ((count = fread(buffer,1,sizeof buffer,temporary)))
                if (fwrite(buffer,1,count,output) != count) { result = sg_fail(&diagnostic,SG_IO,0,"output copy failed"); break; }
            if (ferror(temporary)) result = sg_fail(&diagnostic,SG_IO,0,"temporary output read failed");
            if (fclose(output)) result = sg_fail(&diagnostic,SG_IO,0,"output close failed");
            if (!result && rename(staging_name,arguments[4])) result = sg_fail(&diagnostic,SG_IO,0,"atomic output replacement failed");
        }
        if (staging_name) { if (result) unlink(staging_name); free(staging_name); }
    }
    if (temporary) fclose(temporary);
    if (result) fprintf(stderr,"FAIL code=%d point=%d %s\n",result,diagnostic.point,diagnostic.message);
    else fprintf(stderr,"PASS %d poses, %d joints; modeled trajectory only\n",path->count,machine.count);
    free(path); free(plan); return result;
}
