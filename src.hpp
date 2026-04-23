#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    /////////////////////////////////
    /// TODO: You can add any [private] member variable or [private] member function you need.
    /////////////////////////////////
    
    // Check if a velocity will cause collision with another robot
    bool will_collide(const Vec &v_test, int other_id, double time_interval = 0.1, double safety_margin = 0.5) {
        Vec other_pos = monitor->get_pos_cur(other_id);
        Vec other_v = monitor->get_v_cur(other_id);
        double other_r = monitor->get_r(other_id);
        
        Vec delta_pos = pos_cur - other_pos;
        Vec delta_v = v_test - other_v;
        
        // If moving away, no collision
        double project = delta_pos.dot(delta_v);
        if (project >= 0) {
            return false;
        }
        
        project /= -delta_v.norm();
        double min_dis_sqr;
        double delta_r = r + other_r + safety_margin;
        
        if (project < delta_v.norm() * time_interval) {
            min_dis_sqr = delta_pos.norm_sqr() - project * project;
        } else {
            min_dis_sqr = (delta_pos + delta_v * time_interval).norm_sqr();
        }
        
        return min_dis_sqr <= delta_r * delta_r - 0.01;
    }
    
    // Check if velocity is safe (no collision with any robot)
    bool is_safe_velocity(const Vec &v_test, double safety_margin = 0.5) {
        if (v_test.norm() > v_max + 0.001) {
            return false;
        }
        
        int robot_num = monitor->get_robot_number();
        for (int i = 0; i < robot_num; ++i) {
            if (i == id) continue;
            if (will_collide(v_test, i, 0.1, safety_margin)) {
                return false;
            }
        }
        return true;
    }

public:

    Vec get_v_next() {
        /// TODO: You need to decide the speed of the robot at the next moment.
        ///       You can obtain information about the robot being processed in the member variable of class Controller.
        ///       You can obtain information about the other robot by accessing the interface of *monitor.
        /// Warning: You cannot use any static variable or global variable!
        ///          You should not try to output any information!
        ///          You cannot modify any code that is not allowed to be modified!
        ///          All illegal behavior will be voided.
        
        // Calculate desired velocity toward target
        Vec to_target = pos_tar - pos_cur;
        double dist_to_target = to_target.norm();
        
        // If already at target, stop
        if (dist_to_target < 0.01) {
            return Vec(0, 0);
        }
        
        // Check for nearby robots and potential conflicts
        int robot_num = monitor->get_robot_number();
        bool has_conflict = false;
        int conflict_robot = -1;
        double min_conflict_dist = 1e9;
        
        for (int i = 0; i < robot_num; ++i) {
            if (i == id) continue;
            
            Vec other_pos = monitor->get_pos_cur(i);
            Vec delta = pos_cur - other_pos;
            double dist = delta.norm();
            double other_r = monitor->get_r(i);
            
            if (dist < min_conflict_dist && dist < (r + other_r) * 3.0) {
                has_conflict = true;
                conflict_robot = i;
                min_conflict_dist = dist;
            }
        }
        
        // Desired velocity: move toward target at max speed
        Vec v_desired = to_target.normalize() * v_max;
        
        // If desired velocity is safe, use it
        if (is_safe_velocity(v_desired, 0.5)) {
            return v_desired;
        }
        
        // Try slower velocities toward target
        for (double speed_factor = 0.9; speed_factor > 0.1; speed_factor -= 0.1) {
            Vec v_test = to_target.normalize() * (v_max * speed_factor);
            if (is_safe_velocity(v_test, 0.5)) {
                return v_test;
            }
        }
        
        // If there's a conflict, implement priority-based avoidance
        if (has_conflict && conflict_robot >= 0) {
            Vec other_pos = monitor->get_pos_cur(conflict_robot);
            Vec delta = pos_cur - other_pos;
            
            // Lower ID has priority to go straight, higher ID yields
            if (id > conflict_robot) {
                // This robot should yield - try to go around
                Vec perp1 = Vec(-to_target.y, to_target.x).normalize();
                Vec perp2 = Vec(to_target.y, -to_target.x).normalize();
                
                // Choose perpendicular direction that moves away from conflict
                Vec avoid_dir = perp1;
                if (delta.dot(perp2) > delta.dot(perp1)) {
                    avoid_dir = perp2;
                }
                
                // Try moving perpendicular + forward
                for (double angle_factor = 0.9; angle_factor >= 0.1; angle_factor -= 0.1) {
                    for (double speed = v_max; speed > 0.1; speed -= v_max * 0.15) {
                        Vec v_test = (avoid_dir * angle_factor + to_target.normalize() * (1.0 - angle_factor)).normalize() * speed;
                        if (is_safe_velocity(v_test, 0.2)) {
                            return v_test;
                        }
                    }
                }
                
                // Try pure perpendicular
                for (double speed = v_max * 0.7; speed > 0.1; speed -= v_max * 0.15) {
                    if (is_safe_velocity(avoid_dir * speed, 0.2)) {
                        return avoid_dir * speed;
                    }
                }
                
                // Try stopping
                if (is_safe_velocity(Vec(0, 0), 0.0)) {
                    return Vec(0, 0);
                }
            }
        }
        
        // Try with smaller safety margin
        for (double speed_factor = 1.0; speed_factor > 0.1; speed_factor -= 0.1) {
            Vec v_test = to_target.normalize() * (v_max * speed_factor);
            if (is_safe_velocity(v_test, 0.2)) {
                return v_test;
            }
        }
        
        // Try perpendicular directions to avoid head-on collision
        Vec perp1 = Vec(-to_target.y, to_target.x).normalize();
        Vec perp2 = Vec(to_target.y, -to_target.x).normalize();
        
        // Use ID to break symmetry
        bool prefer_perp1 = (id % 2 == 0);
        
        for (double speed = v_max; speed > 0.1; speed -= v_max * 0.1) {
            // Try perpendicular + forward
            Vec v_test1 = (perp1 * 0.7 + to_target.normalize() * 0.3).normalize() * speed;
            Vec v_test2 = (perp2 * 0.7 + to_target.normalize() * 0.3).normalize() * speed;
            
            if (prefer_perp1) {
                if (is_safe_velocity(v_test1, 0.3)) return v_test1;
                if (is_safe_velocity(v_test2, 0.3)) return v_test2;
            } else {
                if (is_safe_velocity(v_test2, 0.3)) return v_test2;
                if (is_safe_velocity(v_test1, 0.3)) return v_test1;
            }
        }
        
        // Try pure perpendicular
        for (double speed = v_max * 0.8; speed > 0.1; speed -= v_max * 0.1) {
            if (prefer_perp1) {
                if (is_safe_velocity(perp1 * speed, 0.3)) return perp1 * speed;
                if (is_safe_velocity(perp2 * speed, 0.3)) return perp2 * speed;
            } else {
                if (is_safe_velocity(perp2 * speed, 0.3)) return perp2 * speed;
                if (is_safe_velocity(perp1 * speed, 0.3)) return perp1 * speed;
            }
        }
        
        // Try to find alternative directions with repulsion
        Vec avoidance_dir(0, 0);
        
        // Calculate avoidance direction based on nearby robots
        for (int i = 0; i < robot_num; ++i) {
            if (i == id) continue;
            
            Vec other_pos = monitor->get_pos_cur(i);
            Vec delta = pos_cur - other_pos;
            double dist = delta.norm();
            double other_r = monitor->get_r(i);
            double safe_dist = r + other_r + 3.0;
            
            if (dist < safe_dist && dist > 0.01) {
                // Add repulsion force
                Vec repulsion = delta.normalize() * (safe_dist / (dist + 0.1));
                avoidance_dir += repulsion;
            }
        }
        
        // Combine target direction and avoidance direction
        if (avoidance_dir.norm() > 0.01) {
            Vec combined_dir = to_target.normalize() * 0.4 + avoidance_dir.normalize() * 0.6;
            
            for (double speed = v_max; speed > 0.1; speed -= v_max * 0.1) {
                Vec v_test = combined_dir.normalize() * speed;
                if (is_safe_velocity(v_test, 0.2)) {
                    return v_test;
                }
            }
        }
        
        // Last resort: try to stop
        if (is_safe_velocity(Vec(0, 0), 0.0)) {
            return Vec(0, 0);
        }
        
        // If nothing works, return zero velocity
        return Vec(0, 0);
    }
};


/////////////////////////////////
/// TODO: You can add any class or struct you need.
/////////////////////////////////


#endif //PPCA_SRC_HPP
