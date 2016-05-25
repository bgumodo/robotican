//
// Created by tom on 18/05/16.
//

#ifndef ROBOTICAN_HARDWARE_INTERFACE_JOINTINFO_H
#define ROBOTICAN_HARDWARE_INTERFACE_JOINTINFO_H

namespace robotican_hardware {
    struct JointInfo_t {
        double position;
        double effort;
        double velocity;
        double cmd;

        JointInfo_t() {
            position = effort = velocity = cmd = 0;
        }
    };
}

#endif //ROBOTICAN_HARDWARE_INTERFACE_JOINTINFO_H
