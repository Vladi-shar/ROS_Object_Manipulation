#include "mov_box_node.h"

BoxMover::direction_enum direction_string_to_enum(std::string s) {
    if (s.compare("left") == 0)
        return BoxMover::left;
    if (s.compare("right") == 0)
        return BoxMover::right;
    if (s.compare("up") == 0)
        return BoxMover::forward;
    if (s.compare("down") == 0)
        return BoxMover::backward;
}


BoxMover::BoxMover(){};

BoxMover::~BoxMover() {
    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}
bool BoxMover::init(int argc_, char** argv_) {
    ros::init(argc_, argv_, "mov_boxes_nodes");
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    ini_pose_msg_pub_ = n.advertise<std_msgs::String>("/robotis/base/ini_pose_msg", 0);
    set_mode_msg_pub_ = n.advertise<std_msgs::String>("/robotis/base/set_mode_msg", 0);

    kinematics_pose_msg_pub_ = n.advertise<manipulator_h_base_module_msgs::KinematicsPose>(
            "/robotis/base/kinematics_pose_msg", 0);

    left_lower_finger_msg_pub_   = n.advertise<std_msgs::Float64>("/robotis_manipulator_h/rh_l1_position/command",0);
    right_lower_finger_msg_pub_  = n.advertise<std_msgs::Float64>("/robotis_manipulator_h/rh_p12_rn_position/command",0);
    left_upper_finger_msg_pub_   = n.advertise<std_msgs::Float64>("/robotis_manipulator_h/rh_l2_position/command",0);
    right_upper_finger_msg_pub_  = n.advertise<std_msgs::Float64>("/robotis_manipulator_h/rh_r2_position/command",0);

    get_kinematics_pose_client_ = n.serviceClient<manipulator_h_base_module_msgs::GetKinematicsPose>(
            "/robotis/base/get_kinematics_pose", 0);

    status_msg_sub_ = n.subscribe("/robotis/status", 10, &BoxMover::statusMsgCallback, this);
    std::cout << "Setting Mode..." << std::endl;
    set_mode();
    ros::Duration(1).sleep();
    ros::spinOnce();
    std::cout << "Starting!" << std::endl;
    init_pos();
    return true;
}
void BoxMover::run() {

    ros::Rate loop_rate(50);

    while ( ros::ok() )
    {
        ros::spinOnce();

        loop_rate.sleep();
    }
    //std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    //Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

int BoxMover::run_instructions(std::string s) {
    std::regex rgx("\\((\\d), (\\d)\\),\\s*'([a-z]+)',\\s*(\\d)");
    std::smatch matches;
    std::smatch matches2;
    BoxMover::direction_enum direction;
    BoxMover::direction_enum direction2;
    int x, y, next_x, next_y;
    
   
   
   
   
    while (std::regex_search(s, matches, rgx)) {
        //std::cout << "Match found\n";
        const char *fst_match = matches.str(1).c_str();
        const char *sec_match = matches.str(2).c_str();
        const char *trd_match = matches.str(3).c_str();

        x = std::stoi(sec_match);
        y = std::stoi(fst_match);
        direction = direction_string_to_enum(trd_match);
        s = matches.suffix().str();
        if (std::regex_search(s, matches2, rgx)){
            const char *fst_match2 = matches.str(1).c_str();
            const char *sec_match2 = matches.str(2).c_str();
            const char *trd_match2 = matches.str(3).c_str();
            next_x = std::stoi(sec_match);
            next_y = std::stoi(fst_match);
            direction = direction_string_to_enum(trd_match2);
            perform_insctruction(x, y, direction, next_x, next_y, direction2);
        }
        else{
            perform_insctruction(x, y, direction, 0, 0, direction);
        }
    }
    init_pos();
}

int BoxMover::perform_insctruction(int x, int y, direction_enum dir, int next_x, int next_y, direction_enum dir2) {
    //printf("Got instruction x: %d, y: %d, direction: %d\n", x, y, dir);
    //#############################before moving
    int target_x = x;
    int target_next_x = next_x;
    int target_y = y;
    int target_next_y = next_y;
    align_enum align;
    switch (dir) {
        case left:
            target_y = y + 1;
            align = left_align;
            break;
        case right:
            target_y = y - 1;
            align = right_align;
            break;
        case forward:
            target_x = x + 1;
            align = top;
            break;
        default:
            printf("Illegal direction %d\n", dir);
            return 0;
    }
    // moving the head near the object
    std::cout << "Approaching the object..." << std::endl;

    mov_head_to_y_then_x(target_x, target_y, center);

    std::cout << "Moving the object..." << std::endl;

    mov_head_to_y_then_x(x, y, align);

    std::cout << "Retreating..." << std::endl;
    mov_head_to_x_then_y(target_x, target_y, center);

    if (next_x || next_y){
        switch (dir2) {
            case left:
                target_next_y = y + 1;
                break;
            case right:
                target_next_y = y - 1;
                break;
            case forward:
                target_next_x = x + 1;
                break;
            default:
                printf("Illegal direction %d\n", dir);
                return 0;
        }
        if (next_x != target_next_x || next_y != target_next_y){
            init_pos();
        }
    }
    head_row = 5;
    head_col = 2;
    head_x = 0.191;
    head_y = 0;
    head_z = 0.19;
    head_align = center;
}

int BoxMover::mov_head_to_y_then_x(int x, int y, align_enum align) {
    //printf("mov_head_to x: %d y: %d \n", x, y);
    //printf("mov_head_to head_row: %d head_col: %d \n", head_row, head_col);

    //moving on y axis
    if (y - head_col != 0) {
        if (y < head_col) {
            mov_left(head_col - y, align);
        } else {
            mov_right(y - head_col, align);
        }
    }
    //moving on x axis
    if (x - head_row != 0) {
        if (x < head_row) {
            mov_forward(head_row - x, align);
        } else {
            mov_backward(x - head_row, align);
        }
    }
}

int BoxMover::mov_head_to_x_then_y(int x, int y, align_enum align) {
    //printf("mov_head_to x: %d y: %d \n", x, y);
    //printf("mov_head_to head_row: %d head_col: %d \n", head_row, head_col);

    // moving on x axis
    if (x - head_row != 0) {
        if (x < head_row) {
            mov_forward(head_row - x, align);
        } else {
            mov_backward(x - head_row, align);
        }
    }
    //moving on y axis
    if (y - head_col != 0) {
        if (y < head_col) {
            mov_left(head_col - y, align);
        } else {
            mov_right(y - head_col, align);
        }
    }

}

float BoxMover::get_offset_y(align_enum align) {

    if (align == head_align) {
        return 0;
    }

    float offset = 0;
    switch (align) {
        case left_align:
            if (head_align == right_align)
                offset = +cell_length;
            else // head_align = center
                offset = +cell_length / 2;
            break;
        case right_align:
            if (head_align == left_align)
                offset = -cell_length;
            else // head_align = center
                offset = -cell_length / 2;
            break;
        case center:
            if (head_align == left_align)
                offset = -cell_length / 2;
            else // head_align = right
                offset = +cell_length / 2;
            break;
            /* code */
        default:
            printf("float get_offset_y(align_enum align) - got illegal align value : %d\n", align);
            break;
    }

    return offset;

}

float BoxMover::get_offset_x(align_enum align) {
    if (align == head_align) {
        return 0;
    }
    float offset = 0;
    switch (align) {
        case top:
            // head_align = center
            offset = +cell_length / 2;
            break;
        case center:
            //head_align == top
            offset = -cell_length / 2;
            break;

        default:
            printf("float get_offset_x(align_enum align) - got illegal align value : %d\n", align);
            break;
    }

    return offset;

}

int BoxMover::mov_forward(int units, align_enum align) {
    flag = 1;
    head_row -= units;
    float x_target = 0;
    float offset = get_offset_x(align);
    x_target = units * cell_length + head_x + offset;
    head_align = align;
    send_mov_message(x_target, head_y, head_z);
    waitForEndTragectory();
    return 1;
}

int BoxMover::mov_backward(int units, align_enum align) {
    flag = 1;
    head_row += units;
    float x_target = 0;
    float offset = get_offset_x(align);
    x_target = units * (-cell_length) + head_x + offset;
    head_align = align;
    send_mov_message(x_target, head_y, head_z);
    waitForEndTragectory();
    return 1;
}

int BoxMover::mov_left(int units, align_enum align) {
    flag = 1;
    head_col -= units;
    float y_target = 0;
    float offset = get_offset_y(align);
    y_target = units * cell_length + head_y + offset;
    head_align = align;
    send_mov_message(head_x, y_target, head_z);
    waitForEndTragectory();
    return 1;
}

int BoxMover::mov_right(int units, align_enum align) {
    flag = 1;
    head_col += units;
    float y_target = 0;
    float offset = get_offset_y(align);
    y_target = units * (-cell_length) + head_y + offset;
    head_align = align;
    send_mov_message(head_x, y_target, head_z);
    waitForEndTragectory();
    return 1;
}

void BoxMover::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg) {
    if (msg->status_msg == "End Trajectory") {
        flag = 0;
    }
}

int BoxMover::init_pos() {

    send_mov_message(0.191, 0.000, 0.19);
    return 1;
}

int BoxMover::set_mode() {
    ros::Rate loop_rate(50);
    loop_rate.sleep();
    std_msgs::String msg;
    msg.data = "set_mode";
    set_mode_msg_pub_.publish(msg);
    return 1;
}

int BoxMover::send_mov_message(float x, float y, float z) {
    flag = 1;
    printf("send_mov_message x: %f y: %f z: %f \n", x, y, z);
    head_x = x;
    head_y = y;
    head_z = z;
    ros::Rate loop_rate(50);
    loop_rate.sleep();
    manipulator_h_base_module_msgs::KinematicsPose msg;

    msg.name = "arm";

    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    //  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;


    kinematics_pose_msg_pub_.publish(msg);
    waitForEndTragectory();
    return 1;
}

int BoxMover::mov_finger(std_msgs::Float64 finger_value, finger_enum finger){

    ros::Publisher publisher;
    switch (finger)
    {
    case left_upper:
        publisher =left_upper_finger_msg_pub_;
        break;
     case left_lower:
        publisher =left_lower_finger_msg_pub_;
        break;

     case right_upper:
       publisher = right_upper_finger_msg_pub_;
        break;

         case right_lower:
      publisher =  right_lower_finger_msg_pub_;
        break;
    
    default:
        printf("No finger");
        break;
    }
    if (publisher){
        publisher.publish(finger_value);
    }
    return 1;
}

int BoxMover::mov_fingers(float left_uper, float left_lower, float right_upper,float right_lower){
     ros::Rate loop_rate(50);
        std_msgs::Float64 left_uper64 , left_lower64, right_upper64, right_lower64;
        left_uper64.data = left_uper;
        left_lower64.data = left_lower;
        right_upper64.data = right_upper;
        right_lower64.data = right_lower;
    mov_finger (left_uper64, BoxMover::left_upper);
    loop_rate.sleep();
    mov_finger (left_lower64, BoxMover::left_lower);
     loop_rate.sleep();
    mov_finger (right_upper64, BoxMover::right_upper);
     loop_rate.sleep();
    mov_finger (right_lower64, BoxMover::right_lower);
     loop_rate.sleep();
}


int BoxMover::set_gripper_mov_object_pos(){
 
    
  mov_fingers(0.5,0.5,0.5,0.5);
}
int BoxMover::open_gripper(){
    mov_fingers(1.0, 0.0, 1.0, 0.0);
}
int BoxMover::close_griper(){
     mov_fingers(0.0, 0.4, 0.0, 0.4);
}
void BoxMover::waitForEndTragectory(){
    while (flag) {
        ros::Duration(1).sleep();
        ros::spinOnce();
    }
}


int main(int argc, char **argv) {

    BoxMover mover;
    mover.init(argc, argv);
    mover.set_gripper_mov_object_pos();   
    std::cout << "The argv[1]\n" << argv[1] << std::endl;
   
     std::string s =argv[1];
     mover.run_instructions(s);
     mover.open_gripper();
    
    mover.send_mov_message(0.640, 0.0, 0.190);
    mover.close_griper();
    mover.send_mov_message(0.640, 0.0, 0.400);
    return 0;
}
