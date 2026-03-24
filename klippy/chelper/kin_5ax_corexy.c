// CoreXY kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

static double
five_ax_corexy_stepper_plus_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.x + c.y;
}

static double
five_ax_corexy_stepper_minus_calc_position(struct stepper_kinematics *sk, struct move *m
                                   , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    return c.x - c.y;
}

static double
polar_stepper_angle_calc_position(struct stepper_kinematics *sk, struct move *m
                                  , double move_time)
{
    struct coord c = move_get_coord(m, move_time);
    // XXX - handle x==y==0
    double angle = atan2(c.y, c.x);
    if (angle - sk->commanded_pos > M_PI)
        angle -= 2. * M_PI;
    else if (angle - sk->commanded_pos < -M_PI)
        angle += 2. * M_PI;
    return angle;
}

static void
polar_stepper_angle_post_fixup(struct stepper_kinematics *sk)
{
    // Normalize the stepper_bed angle
    if (sk->commanded_pos < -M_PI)
        sk->commanded_pos += 2 * M_PI;
    else if (sk->commanded_pos > M_PI)
        sk->commanded_pos -= 2 * M_PI;
}


struct stepper_kinematics * __visible
five_ax_corexy_stepper_alloc(char type)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (type == '+') {
        sk->calc_position_cb = five_ax_corexy_stepper_plus_calc_position;
    } else if (type == '-') {
        sk->calc_position_cb = five_ax_corexy_stepper_minus_calc_position;
    } else if (type == 'z') {
        sk->calc_position_cb = five_ax_polar_stepper_angle_calc_position;
        sk->post_cb = polar_stepper_angle_post_fixup;
    } else if (type == 'u') {
        sk->calc_position_cb = five_ax_polar_stepper_angle_calc_position;
        sk->post_cb = polar_stepper_angle_post_fixup;
    } else if (type == 'v') {
        sk->calc_position_cb = five_ax_polar_stepper_angle_calc_position;
        sk->post_cb = polar_stepper_angle_post_fixup;
    }
    sk->active_flags = AF_X | AF_Y;
    return sk;
}
