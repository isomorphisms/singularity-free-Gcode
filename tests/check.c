#define _POSIX_C_SOURCE 200809L
#include "gcode_planner.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>

static int checks, failures;
#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; fprintf(stderr,"FAIL %s:%d %s\n",__FILE__,__LINE__,#condition); } } while (0)

/* Independent scalar forward equations: no production matrix/vector helpers.
   The table inverse first undoes E about z, then A about x. */
static sg_pose independent_forward(const sg_machine *machine, const double *joint) {
    double cosine_b = cos(joint[3]), sine_b = sin(joint[3]);
    double cosine_c = cos(joint[4]), sine_c = sin(joint[4]);
    double x = joint[0], y = joint[1], z = joint[2];
    double direction_x = sine_b*cosine_c, direction_y = sine_b*sine_c, direction_z = cosine_b;
    if (machine->count == 12) { x += joint[5]-joint[8]; y += joint[6]-joint[9]; z += joint[7]; }
    x -= machine->tool_length*direction_x; y -= machine->tool_length*direction_y; z -= machine->tool_length*direction_z;
    if (machine->count == 12) {
        double cosine_e = cos(joint[11]), sine_e = sin(joint[11]);
        double cosine_a = cos(joint[10]), sine_a = sin(joint[10]);
        double first_x = cosine_e*x+sine_e*y, first_y = -sine_e*x+cosine_e*y;
        x = first_x; y = cosine_a*first_y+sine_a*z; z = -sine_a*first_y+cosine_a*z;
        first_x = cosine_e*direction_x+sine_e*direction_y;
        first_y = -sine_e*direction_x+cosine_e*direction_y;
        direction_x = first_x;
        double previous_z = direction_z;
        direction_y = cosine_a*first_y+sine_a*previous_z;
        direction_z = -sine_a*first_y+cosine_a*previous_z;
    }
    sg_pose answer = {0};
    answer.position = (sg_vector){x,y,z}; answer.direction = (sg_vector){direction_x,direction_y,direction_z};
    double columns[3][3] = {{cosine_c*cosine_b,sine_c*cosine_b,-sine_b},
        {-sine_c,cosine_c,0},{cosine_c*sine_b,sine_c*sine_b,cosine_b}};
    for (int column = 0; column < 3; ++column) {
        double frame_x = columns[column][0], frame_y = columns[column][1], frame_z = columns[column][2];
        if (machine->count==12) {
            double rotated_x = cos(joint[11])*frame_x+sin(joint[11])*frame_y;
            double rotated_y = -sin(joint[11])*frame_x+cos(joint[11])*frame_y;
            frame_x = rotated_x;
            frame_y = cos(joint[10])*rotated_y+sin(joint[10])*frame_z;
            frame_z = -sin(joint[10])*rotated_y+cos(joint[10])*frame_z;
        }
        answer.rotation.entry[0][column] = frame_x;
        answer.rotation.entry[1][column] = frame_y;
        answer.rotation.entry[2][column] = frame_z;
    }
    return answer;
}
static double distance(sg_vector first, sg_vector second) {
    return hypot(hypot(first.x-second.x,first.y-second.y),first.z-second.z);
}
static void set_initial(sg_machine *machine, const double *joint) {
    for (int axis = 0; axis < machine->count; ++axis) machine->axes[axis].initial = joint[axis];
}
static void point_from_joint(sg_toolpath *path, const sg_machine *machine, const double *joint) {
    sg_waypoint *point = &path->points[path->count++]; memset(point,0,sizeof *point);
    point->pose = independent_forward(machine,joint); point->seconds = 0.5;
    snprintf(point->object,sizeof point->object,"test-object"); snprintf(point->operation,sizeof point->operation,"test-motion");
}

static void check_forward_and_jacobian(void) {
    sg_machine machine;
    for (int count = 5; count <= 12; count += 7) {
        sg_default_machine(&machine,count);
        for (int sample = 0; sample < 80; ++sample) {
            double joint[SG_AXES] = {0};
            for (int axis = 0; axis < count; ++axis)
                joint[axis] = machine.axes[axis].rotary ? 0.7*sin(sample*0.2+axis) : 40*cos(sample*0.1+axis);
            sg_pose actual = sg_forward(&machine,joint), expected = independent_forward(&machine,joint);
            CHECK(distance(actual.position,expected.position)<1e-10);
            CHECK(distance(actual.direction,expected.direction)<1e-12);
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    CHECK(fabs(actual.rotation.entry[row][column]-expected.rotation.entry[row][column])<1e-12);
            if (count==5) {
                /* LinuxCNC v2.9.4 5axiskins equation parity, not execution of
                   its binary: B_linux=-B and Z_linux=Z-L, W_linux=0. */
                double linux_b_degrees = -joint[3]*180/SG_PI;
                double polar_angle = (180-linux_b_degrees)*SG_PI/180;
                double radial_x = machine.tool_length*sin(polar_angle)*cos(joint[4]);
                double radial_y = machine.tool_length*sin(polar_angle)*sin(joint[4]);
                double radial_z = machine.tool_length*cos(polar_angle);
                sg_vector linux_tip = {joint[0]+radial_x,joint[1]+radial_y,(joint[2]-machine.tool_length)+machine.tool_length+radial_z};
                CHECK(distance(actual.position,linux_tip)<1e-10);
            }
            double jacobian[6][SG_AXES]; sg_jacobian(&machine,joint,0,jacobian);
            for (int axis = 0; axis < count; ++axis) {
                double plus[SG_AXES], minus[SG_AXES]; memcpy(plus,joint,sizeof plus); memcpy(minus,joint,sizeof minus);
                double step = machine.axes[axis].rotary ? 3e-6 : 3e-4;
                plus[axis] += step; minus[axis] -= step;
                sg_pose forward = independent_forward(&machine,plus), backward = independent_forward(&machine,minus);
                double scale = machine.axes[axis].rotary ? 1 : machine.tool_length;
                CHECK(fabs(jacobian[0][axis]-(forward.position.x-backward.position.x)*scale/(2*step*machine.tool_length))<1e-7);
                CHECK(fabs(jacobian[1][axis]-(forward.position.y-backward.position.y)*scale/(2*step*machine.tool_length))<1e-7);
                CHECK(fabs(jacobian[2][axis]-(forward.position.z-backward.position.z)*scale/(2*step*machine.tool_length))<1e-7);
            }
        }
    }
    sg_default_machine(&machine,5); double joint[SG_AXES] = {0,0,130,0.4,0.2};
    CHECK(sg_analyze(&machine,joint,0,0).rank==5);
    joint[3] = 0;
    CHECK(sg_analyze(&machine,joint,0,0).rank==4);
    CHECK(sg_analyze(&machine,joint,1,0).rank==5); /* full frame has no pole rank drop */
    CHECK(sg_analyze(&machine,joint,0,0).minimum_singular_value<1e-10);
    joint[3] = 1e-8;
    sg_condition near = sg_analyze(&machine,joint,0,0);
    CHECK(near.minimum_singular_value>1e-10 && near.minimum_singular_value<1e-7);
    sg_default_machine(&machine,12); joint[3] = 0;
    CHECK(sg_analyze(&machine,joint,0,0).rank==5);
    CHECK(sg_analyze(&machine,joint,1,0).rank==6);
    joint[4] = SG_PI/2;
    CHECK(sg_analyze(&machine,joint,0,0).rank==4);
    CHECK(sg_analyze(&machine,joint,1,0).rank==5); /* genuine full F rank loss */
    joint[4] = SG_PI/2-1e-8;
    sg_condition twelve_near = sg_analyze(&machine,joint,1,0);
    CHECK(twelve_near.minimum_singular_value>1e-12 && twelve_near.minimum_singular_value<1e-7);
    joint[4] = 0.2;
    sg_pose before = sg_forward(&machine,joint);
    joint[0] += 12; joint[5] -= 12; joint[1] += 5; joint[9] += 5;
    CHECK(distance(before.position,sg_forward(&machine,joint).position)<1e-10);
    CHECK(distance(before.direction,sg_forward(&machine,joint).direction)<1e-12);
    sg_default_machine(&machine,5); memset(joint,0,sizeof joint); joint[2] = 130; joint[0] = machine.axes[0].upper;
    double desired[5] = {0.2,0,0,0,0}, velocity[SG_AXES] = {0};
    CHECK(sg_velocity_residual(&machine,joint,desired,velocity)>0.05);
    CHECK(velocity[0]<=1e-12);
    desired[0] = -0.2;
    CHECK(sg_velocity_residual(&machine,joint,desired,velocity)<1e-7);
    CHECK(velocity[0]<0);
    CHECK(sg_analyze(&machine,joint,0,1).rank<5);
}

static void check_gcode_independently(const sg_machine *machine, const sg_toolpath *path, const sg_plan *plan) {
    FILE *output = tmpfile(); sg_diagnostic diagnostic = {0}; CHECK(output!=NULL); if (!output) return;
    CHECK(!sg_write_linuxcnc(output,machine,path,plan,&diagnostic)); rewind(output);
    char line[1024]; int moves = 0, inverse_time = 0, exact_stop = 0;
    while (fgets(line,sizeof line,output)) {
        if (!strcmp(line,"G93\n")) inverse_time = 1;
        if (strstr(line,"G61")) exact_stop = 1;
        if (line[0] != 'N') continue;
        int block; double x,y,z,b,c,feed; char remainder;
        CHECK(sscanf(line,"N%d G1 X%lf Y%lf Z%lf B%lf C%lf F%lf %c",&block,&x,&y,&z,&b,&c,&feed,&remainder)==7);
        ++moves; CHECK(block==moves*10 && feed>0);
        double joint[SG_AXES] = {x,y,z,b*SG_PI/180,c*SG_PI/180};
        sg_pose expected = independent_forward(machine,joint);
        CHECK(distance(expected.position,path->points[moves].pose.position)<1e-7);
        CHECK(distance(expected.direction,path->points[moves].pose.direction)<1e-8);
        double required = sg_segment_seconds(machine,plan->configuration[moves-1].joint,joint,path->points[moves].seconds)/(1+1e-9);
        CHECK(60/feed+1e-7>=required);
    }
    CHECK(moves==path->count-1 && inverse_time && exact_stop);
    fclose(output);
}

static void check_branches_and_paths(void) {
    sg_machine machine; sg_default_machine(&machine,5);
    sg_toolpath *path = calloc(1,sizeof *path); sg_plan *plan = malloc(sizeof *plan), *baseline = malloc(sizeof *baseline);
    CHECK(path && plan && baseline); if (!path || !plan || !baseline) goto finish;
    double joint[SG_AXES] = {0,0,130,0.05,0.3}; set_initial(&machine,joint);
    /* Keep the intended tip position fixed while crossing the pole. */
    sg_pose initial = independent_forward(&machine,joint);
    for (int point = 0; point <= 20; ++point) {
        double tilt = 0.05-0.005*point;
        joint[3] = tilt;
        joint[0] = initial.position.x+machine.tool_length*sin(tilt)*cos(0.3);
        joint[1] = initial.position.y+machine.tool_length*sin(tilt)*sin(0.3);
        joint[2] = initial.position.z+machine.tool_length*cos(tilt);
        point_from_joint(path,&machine,joint);
    }
    sg_diagnostic diagnostic = {0};
    CHECK(!sg_plan_path(&machine,path,plan,&diagnostic));
    CHECK(!sg_principal_baseline(&machine,path,baseline,&diagnostic));
    sg_metrics after = sg_measure(&machine,path,plan), before = sg_measure(&machine,path,baseline);
    CHECK(after.invalid_edges==0 && before.invalid_edges>0);
    CHECK(after.travel[4]<1e-6 && before.travel[4]>1);
    CHECK(plan->configuration[20].joint[3]<0);
    CHECK(after.minimum_singular_value<1e-9);
    for (int axis = 0; axis < machine.count; ++axis) {
        CHECK(after.maximum_velocity[axis]<=machine.axes[axis].velocity+1e-7);
        CHECK(after.maximum_acceleration[axis]<=machine.axes[axis].acceleration+1e-7);
        CHECK(after.maximum_jerk[axis]<=machine.axes[axis].jerk+1e-7);
    }
    check_gcode_independently(&machine,path,plan);
    sg_configuration candidates[SG_CANDIDATES];
    int count = sg_inverse_candidates(&machine,path,1,candidates,&diagnostic), positives = 0, negatives = 0;
    for (int candidate = 0; candidate < count; ++candidate) {
        positives += candidates[candidate].branch==1; negatives += candidates[candidate].branch==-1;
        sg_pose pose = independent_forward(&machine,candidates[candidate].joint);
        CHECK(distance(pose.position,path->points[1].pose.position)<1e-7);
        CHECK(distance(pose.direction,path->points[1].pose.direction)<1e-8);
    }
    CHECK(positives && negatives); /* two disconnected regular task branches */
    /* An explicitly provided duplicate pole waypoint leaves a free rotary
       coordinate. Nearest-point greedy selection chooses C=0 and then fails;
       the full-path search uses that existing waypoint to select C=-60. */
    path->count = 0; sg_default_machine(&machine,5);
    memset(joint,0,sizeof joint); joint[2] = 130;
    point_from_joint(path,&machine,joint); point_from_joint(path,&machine,joint);
    for (int point = 1; point <= 12; ++point) {
        joint[3] = 0.02*point; joint[4] = 120*SG_PI/180;
        joint[0] = 80*sin(joint[3])*cos(joint[4]); joint[1] = 80*sin(joint[3])*sin(joint[4]); joint[2] = 50+80*cos(joint[3]);
        point_from_joint(path,&machine,joint);
    }
    CHECK(!sg_plan_path(&machine,path,plan,&diagnostic));
    CHECK(fabs(plan->configuration[1].joint[4]+SG_PI/3)<1e-8);
    CHECK(plan->configuration[2].joint[3]<0);
    double greedy_start[SG_AXES] = {0,0,130};
    count = sg_inverse_candidates(&machine,path,2,candidates,&diagnostic);
    int greedy_continuations = 0;
    for (int candidate = 0; candidate < count; ++candidate)
        greedy_continuations += sg_edge_valid(&machine,&path->points[1],&path->points[2],greedy_start,candidates[candidate].joint,NULL);
    CHECK(greedy_continuations==0);
    CHECK(plan->count==path->count);
    /* The asymmetric housing distinguishes IK branches even when an
       axisymmetric cutter's tip and direction agree. */
    path->count = 0; sg_default_machine(&machine,5);
    memset(joint,0,sizeof joint); joint[3] = 0.4; joint[0] = 80*sin(0.4); joint[2] = 50+80*cos(0.4);
    point_from_joint(path,&machine,joint); point_from_joint(path,&machine,joint);
    machine.obstacle_count = 1;
    machine.obstacles[0] = (sg_sphere){{joint[0]+10*cos(0.4),0,joint[2]-10*sin(0.4)},1};
    CHECK(!sg_configuration_valid(&machine,joint));
    count = sg_inverse_candidates(&machine,path,1,candidates,&diagnostic);
    positives = negatives = 0;
    for (int candidate = 0; candidate < count; ++candidate) {
        positives += candidates[candidate].branch==1; negatives += candidates[candidate].branch==-1;
    }
    CHECK(!positives && negatives);
    path->count = 0;
    sg_default_machine(&machine,5);
    for (int point = 0; point <= 30; ++point) {
        joint[3] = 0.3; joint[4] = (170+point)*SG_PI/180;
        joint[0] = 80*sin(0.3)*cos(joint[4]); joint[1] = 80*sin(0.3)*sin(joint[4]); joint[2] = 50+80*cos(0.3);
        if (!point) set_initial(&machine,joint);
        point_from_joint(path,&machine,joint);
    }
    CHECK(!sg_plan_path(&machine,path,plan,&diagnostic));
    CHECK(fabs(plan->configuration[30].joint[4]-200*SG_PI/180)<1e-8);
    CHECK(!sg_principal_baseline(&machine,path,baseline,&diagnostic));
    CHECK(sg_measure(&machine,path,baseline).invalid_edges>0);
    check_gcode_independently(&machine,path,plan);
    path->points[1].pose.position.x = 10000;
    CHECK(sg_plan_path(&machine,path,plan,&diagnostic)==SG_UNREACHABLE);
    path->count = 0; sg_default_machine(&machine,5);
    memset(joint,0,sizeof joint); joint[0] = -10; joint[2] = 130; set_initial(&machine,joint); point_from_joint(path,&machine,joint);
    joint[0] = 10; point_from_joint(path,&machine,joint);
    machine.obstacle_count = 1; machine.obstacles[0] = (sg_sphere){{0,0,50},2};
    CHECK(sg_configuration_valid(&machine,(double[SG_AXES]){-10,0,130}));
    CHECK(sg_configuration_valid(&machine,joint));
    CHECK(sg_plan_path(&machine,path,plan,&diagnostic)==SG_NO_CONTINUATION); /* interior collision */
    path->count = 0; sg_default_machine(&machine,12);
    machine.axes[0].lower = -10; machine.axes[0].upper = 10;
    memset(joint,0,sizeof joint); joint[2] = 130; point_from_joint(path,&machine,joint);
    for (int point = 1; point <= 10; ++point) {
        joint[0] = 4*point; point_from_joint(path,&machine,joint);
    }
    CHECK(!sg_plan_path(&machine,path,plan,&diagnostic));
    CHECK(fabs(plan->configuration[10].joint[0])<=10+1e-8);
    CHECK(fabs(plan->configuration[10].joint[5])>1);
    for (int point = 0; point < plan->count; ++point)
        CHECK(distance(independent_forward(&machine,plan->configuration[point].joint).position,path->points[point].pose.position)<1e-7);
    /* A fixed obstruction at the conventional nose endpoint blocks head5.
       Moving the workpiece while allocating the second carriage keeps the
       same relative tool path clear in hybrid12. */
    machine.housing_offset = 5;
    machine.obstacle_count = 1; machine.obstacles[0] = (sg_sphere){{40,0,130},1};
    CHECK(!sg_plan_path(&machine,path,plan,&diagnostic));
    sg_machine fixed_workpiece; sg_default_machine(&fixed_workpiece,5);
    fixed_workpiece.housing_offset = 5;
    fixed_workpiece.obstacle_count = 1; fixed_workpiece.obstacles[0] = machine.obstacles[0];
    CHECK(sg_plan_path(&fixed_workpiece,path,baseline,&diagnostic)==SG_UNREACHABLE);
    FILE *output = tmpfile(); CHECK(output!=NULL);
    if (output) { CHECK(sg_write_linuxcnc(output,&machine,path,plan,&diagnostic)==SG_DIALECT); CHECK(ftell(output)==0); fclose(output); }
finish:
    free(path); free(plan); free(baseline);
}

static void check_parsers_and_refusal(void) {
    sg_toolpath *path = malloc(sizeof *path); sg_machine machine; sg_diagnostic diagnostic = {0};
    CHECK(path!=NULL); if (!path) return;
    const char *invalid[] = {
        "toolpath mm-unit-vector-v1\na b cut 1 nan 0 0 0 0 1\na b cut 1 0 0 0 0 0 1\n",
        "toolpath mm-unit-vector-v1\na b cut 1 0 0 0 0 0 2\na b cut 1 0 0 0 0 0 1\n",
        "toolpath inches\na b cut 1 0 0 0 0 0 1\na b cut 1 0 0 0 0 0 1\n",
        "toolpath mm-unit-vector-v1\na b cut 0 0 0 0 0 0 1\na b cut 1 0 0 0 0 0 1\n",
        "toolpath mm-unit-vector-v1\na (M3) cut 1 0 0 0 0 0 1\na b cut 1 0 0 0 0 0 1\n"
    };
    for (size_t test = 0; test < sizeof invalid/sizeof *invalid; ++test) {
        FILE *input = tmpfile(); CHECK(input!=NULL); if (!input) continue;
        fputs(invalid[test],input); rewind(input); CHECK(sg_read_toolpath(input,path,&diagnostic)==SG_INPUT); fclose(input);
    }
    FILE *input = tmpfile(); CHECK(input!=NULL);
    if (input) { fputs("machine head5 mm-radians-v1\naxis X linear -1 1 1 1 1 0\naxis X linear -1 1 1 1 1 0\n",input); rewind(input); CHECK(sg_read_machine(input,&machine,&diagnostic)==SG_INPUT); fclose(input); }
    /* Exercise the real CLI with a nonzero process exit and preserved output. */
    char folder[] = "/tmp/gcode-refusal-XXXXXX";
    CHECK(mkdtemp(folder)!=NULL);
    char source[256], destination[256]; snprintf(source,sizeof source,"%s/input.sg",folder); snprintf(destination,sizeof destination,"%s/output.ngc",folder);
    input = fopen(source,"w"); CHECK(input!=NULL);
    if (input) { fputs("toolpath mm-unit-vector-v1\na initial travel 1 0 0 50 0 0 1\na impossible cut 1 10000 0 50 0 0 1\n",input); fclose(input); }
    FILE *output = fopen(destination,"w"); CHECK(output!=NULL); if (output) { fputs("KEEP\n",output); fclose(output); }
    pid_t child = fork(); CHECK(child>=0);
    if (!child) { execl("./build/gcode-plan","gcode-plan","linuxcnc","examples/pneumatic-fitting/machine.sg",source,destination,(char *)NULL); _exit(127); }
    if (child>0) { int status = 0; CHECK(waitpid(child,&status,0)==child); CHECK(WIFEXITED(status) && WEXITSTATUS(status)==SG_UNREACHABLE); }
    output = fopen(destination,"r"); CHECK(output!=NULL);
    if (output) { char buffer[32]; CHECK(fgets(buffer,sizeof buffer,output)!=NULL && !strcmp(buffer,"KEEP\n")); fclose(output); }
    unlink(source); unlink(destination); rmdir(folder); free(path);
}

static void check_primary_examples(void) {
    const char *directories[] = {"examples/pneumatic-fitting", "examples/twisted-surface"};
    sg_toolpath *path = malloc(sizeof *path); sg_plan *plan = malloc(sizeof *plan);
    CHECK(path && plan); if (!path || !plan) { free(path); free(plan); return; }
    for (int example = 0; example < 2; ++example) {
        char filename[256]; sg_machine machine; sg_diagnostic diagnostic = {0};
        snprintf(filename,sizeof filename,"%s/machine.sg",directories[example]); FILE *input = fopen(filename,"r");
        CHECK(input!=NULL); if (!input) continue;
        CHECK(!sg_read_machine(input,&machine,&diagnostic)); fclose(input);
        snprintf(filename,sizeof filename,"%s/toolpath.sg",directories[example]); input = fopen(filename,"r");
        CHECK(input!=NULL); if (!input) continue;
        CHECK(!sg_read_toolpath(input,path,&diagnostic)); fclose(input);
        CHECK(!sg_plan_path(&machine,path,plan,&diagnostic));
        for (int point = 0; point < plan->count; ++point) {
            sg_pose pose = independent_forward(&machine,plan->configuration[point].joint);
            CHECK(distance(pose.position,path->points[point].pose.position)<1e-7);
            CHECK(distance(pose.direction,path->points[point].pose.direction)<1e-8);
        }
        check_gcode_independently(&machine,path,plan);
    }
    free(path); free(plan);
}

int main(void) {
    check_forward_and_jacobian(); check_branches_and_paths(); check_parsers_and_refusal(); check_primary_examples();
    printf("%s %d independent checks, %d failures\n",failures?"FAIL":"PASS",checks,failures);
    return failures ? 1 : 0;
}
