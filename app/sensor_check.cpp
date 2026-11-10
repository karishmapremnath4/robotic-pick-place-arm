// Step 2 check: the firmware now reads the arm only through chips.
//
// Every number the firmware acts on from here arrives as bytes on a bus. Two
// questions: does each driver recover the physical quantity its device is
// really measuring, and does the dual position channel actually catch a fault
// that neither channel could catch alone?
#include <cmath>
#include <cstdio>

#include "config.hpp"
#include "drivers/can_node.hpp"
#include "drivers/encoder.hpp"
#include "drivers/imu.hpp"
#include "drivers/laser.hpp"
#include "est/arm_model.hpp"
#include "est/lin.hpp"
#include "sim/sensor_board.hpp"

using namespace ppa;

namespace {

constexpr double kDt = 1.0e-5;
constexpr int kStepsPerService = 100;  // 1 kHz firmware service rate
int g_failures = 0;

void check(const char* what, bool ok, const char* detail) {
    std::printf("  %-46s %s %s\n", what, ok ? "PASS" : "FAIL", detail);
    if (!ok) ++g_failures;
}

const char* status_name(Status s) {
    switch (s) {
        case Status::kOk: return "ok";
        case Status::kTimeout: return "timeout";
        case Status::kBadId: return "bad id";
        case Status::kNack: return "nack";
        case Status::kRangeError: return "range error";
    }
    return "?";
}

// Absolute joint angle from a single turn encoder reading. The commissioning
// offset is subtracted and the result rewrapped, which is the whole job.
float joint_angle_from_encoder(const EncoderSample& s, int joint) {
    return wrap_pi(s.single_turn_rad - kEncoderZeroOffsetRad[joint]);
}

// The firmware side of the cell: buses, drivers, and a crude pose holder so
// the arm can be parked somewhere while readings are compared against truth.
struct Rig {
    arm::PlantParams pp{arm::default_params()};
    World world{pp, 31415u};
    Faults faults{};
    SimClock clk{};
    Rng rng{271828u};
    SensorBoard board{world, faults, clk, rng};

    Imu imu{board.imu_bus};
    Encoder enc1{board.enc1_bus, clk};
    Encoder enc2{board.enc2_bus, clk};
    CanNodes can{board.can_bus, clk};
    LaserRanger laser{board.laser_bus, clk};

    ImuSample last_imu{};
    EncoderSample last_e1{}, last_e2{};

    Status init() {
        Status st = imu.init();
        if (st != Status::kOk) return st;
        st = enc1.init();
        if (st != Status::kOk) return st;
        return enc2.init();
    }

    void service() {
        last_imu = imu.read();
        last_e1 = enc1.read();
        last_e2 = enc2.read();
        can.poll();
        laser.poll();
    }

    // Hold a pose with a crude PD, driving the motors over CAN rather than by
    // poking the plant. The command path is now exercised too.
    void hold(const arm::Vec2& q_ref, double seconds) {
        const int steps = static_cast<int>(seconds / kDt);
        for (int i = 0; i < steps; ++i) {
            if (i % kStepsPerService == 0) {
                service();
                const arm::Vec2 q = world.q();
                const arm::Vec2 qd = world.qd();
                // A parking controller, not the real one. Its only job is to
                // hold the arm still so sensor readings can be compared
                // against truth. Gravity is fed forward so the residual the
                // PD has to handle is small, which keeps the gains low
                // enough not to ring at this 1 kHz rate. A ringing arm
                // saturates the 4 g accelerometer and puts sample lag into
                // every comparison, which is a false alarm this test learned
                // the hard way. The real cascaded loops live in
                // ctrl/joint_axis.hpp and are exercised by ctrl_check.
                const Vec<2> gff = gravity_torque(static_cast<float>(q[0]),
                                                  static_cast<float>(q[1]), 0.0f);
                can.send_drive_current(
                    torque_to_current(0, gff[0]) +
                        static_cast<float>(8.0 * (q_ref[0] - q[0]) - 2.0 * qd[0]),
                    torque_to_current(1, gff[1]) +
                        static_cast<float>(8.0 * (q_ref[1] - q[1]) - 2.0 * qd[1]));
            }
            world.step(faults, kDt);
            clk.advance_us(10);
            board.update();
        }
        service();
    }
};

}  // namespace

int main() {
    char buf[200];
    Rig r;

    // ------------------------------------------------------------ bring up
    std::printf("bring up, over the buses\n");
    const Status st = r.init();
    std::snprintf(buf, sizeof(buf), "%s", status_name(st));
    check("imu identified and configured, encoders zeroed", st == Status::kOk, buf);
    if (st != Status::kOk) return 1;

    const arm::Vec2 q_hold =
        arm::inverse_kinematics(r.pp.arm, arm::Vec2{0.300, -0.020},
                                arm::Elbow::kDown).q;
    r.hold(q_hold, 2.5);

    // Every comparison below assumes the arm has actually settled. If it has
    // not, a stale sample and a moving truth disagree for reasons that have
    // nothing to do with the drivers, which is exactly the false alarm that
    // wasted time on the first run of this test.
    {
        const double speed = std::fabs(r.world.qd()[0]) + std::fabs(r.world.qd()[1]);
        std::snprintf(buf, sizeof(buf), "joint speeds sum to %.5f rad/s", speed);
        check("arm is settled before sensors are compared", speed < 0.02, buf);
    }

    // ------------------------------------------------- SPI joint encoders
    std::printf("\nSPI joint encoders\n");
    {
        const float q1 = joint_angle_from_encoder(r.last_e1, 0);
        const float q2 = joint_angle_from_encoder(r.last_e2, 1);
        const double t1 = r.world.q()[0], t2 = r.world.q()[1];

        std::snprintf(buf, sizeof(buf), "joint 1 %.5f vs truth %.5f rad", q1, t1);
        check("absolute angle recovered after offset", std::fabs(q1 - t1) < 0.002, buf);
        std::snprintf(buf, sizeof(buf), "joint 2 %.5f vs truth %.5f rad", q2, t2);
        check("second joint likewise", std::fabs(q2 - t2) < 0.002, buf);

        std::snprintf(buf, sizeof(buf), "%u parity errors, %u error frames",
                      r.enc1.parity_errors(), r.enc1.error_frames());
        check("no frame errors on a healthy bus",
              r.enc1.parity_errors() == 0 && r.enc1.error_frames() == 0, buf);
    }

    // ---------------------------------------------------- I2C tool imu
    std::printf("\nI2C inertial sensor on the tool\n");
    {
        // At rest the gyro should read nothing, which checks the zero but
        // not the scale.
        std::snprintf(buf, sizeof(buf), "gyro %.4f rad/s at rest", r.last_imu.gyro_z);
        check("tool gyro reads zero when parked",
              std::fabs(r.last_imu.gyro_z) < 0.03f, buf);

        // A stationary tool must read one g, whatever direction it points.
        // The magnitude is the invariant worth checking: it catches a wrong
        // rotation, a missed gravity term, and a scaling error all at once.
        const float mag = std::sqrt(r.last_imu.accel_x * r.last_imu.accel_x +
                                    r.last_imu.accel_y * r.last_imu.accel_y);
        std::snprintf(buf, sizeof(buf), "|a| = %.3f m/s^2, expected 9.807", mag);
        check("stationary tool reads one g", std::fabs(mag - 9.807f) < 0.25f, buf);

        // And it must point the right way: rotating the world frame reading
        // into the tool frame has to reproduce what the device returned.
        const double th = r.world.tool_angle();
        const float ex = static_cast<float>(9.80665 * std::sin(th));
        const float ey = static_cast<float>(9.80665 * std::cos(th));
        std::snprintf(buf, sizeof(buf), "got (%.2f, %.2f), expected (%.2f, %.2f)",
                      r.last_imu.accel_x, r.last_imu.accel_y, ex, ey);
        check("gravity appears in the right tool axes",
              std::fabs(r.last_imu.accel_x - ex) < 0.3f &&
              std::fabs(r.last_imu.accel_y - ey) < 0.3f, buf);
    }

    // -------------------------------------------------------- CAN drives
    std::printf("\nCAN servo drives and gripper\n");
    {
        const DriveStatus& d1 = r.can.drive(0);
        std::snprintf(buf, sizeof(buf), "pos %.4f vs truth %.4f rad",
                      d1.position_rad, r.world.q()[0]);
        check("drive reports its own position",
              d1.valid && std::fabs(d1.position_rad - r.world.q()[0]) < 0.003, buf);
        std::snprintf(buf, sizeof(buf), "current %.4f vs truth %.4f A",
                      d1.current_a, r.world.current()[0]);
        check("drive reports winding current",
              std::fabs(d1.current_a - r.world.current()[0]) < 0.06, buf);

        r.can.send_grip(true);
        r.hold(q_hold, 0.4);
        const GripperStatus& g = r.can.gripper();
        std::snprintf(buf, sizeof(buf), "state %u, clamp %.1f N vs truth %.1f N",
                      static_cast<unsigned>(g.state), g.clamp_n, r.world.grip_force_n());
        check("gripper node reports state and clamp force",
              g.valid && std::fabs(g.clamp_n - r.world.grip_force_n()) < 1.0f, buf);

        std::snprintf(buf, sizeof(buf), "%u unknown frames, queue high water %u of %d",
                      r.can.unknown_frames(), r.board.can_bus.rx_high_water(),
                      SimCan::kRxCapacity);
        check("every frame on the bus was recognised",
              r.can.unknown_frames() == 0 && r.board.can_bus.rx_dropped() == 0, buf);
        r.can.send_grip(false);
        r.hold(q_hold, 0.3);
    }

    // ------------------------------------------------- UART laser ranger
    std::printf("\nUART laser distance sensor\n");
    {
        // The beam is offset 30 mm along x from the tool, so at this pose it
        // is looking down at box 3 rather than at bare table.
        const arm::Vec2 tip = r.world.tip();
        const double beam_x = tip[0] + kLaserOffsetXM;
        const double truth = r.world.distance_below(beam_x, tip[1], r.faults);

        std::snprintf(buf, sizeof(buf), "%u frames parsed, %u checksum errors",
                      r.laser.frames_ok(), r.laser.checksum_errors());
        check("frames reassembled from a byte stream",
              r.laser.frames_ok() > 40 && r.laser.checksum_errors() == 0, buf);

        std::snprintf(buf, sizeof(buf), "%.4f m vs truth %.4f m",
                      r.laser.sample().distance_m, truth);
        check("distance matches what is under the beam",
              r.laser.sample().valid &&
              std::fabs(r.laser.sample().distance_m - truth) < 0.02, buf);

        // Nine bytes at 115200 baud take 0.78 ms, and the firmware polls
        // every 1 ms, so most polls see a partial frame. That the driver
        // framed anything at all is the proof its state machine carries
        // across poll boundaries.
        std::printf("    a frame takes %.2f ms to arrive; the loop polls every 1.00 ms\n",
                    9.0 * SimLaserUart::kByteTimeUs / 1000.0);

        // Too close to see. The blind zone is real on these units, and the
        // strength field is what reveals it: the distance field still holds a
        // plausible looking number.
        //
        // Note the pose. The first attempt at this test lowered the TOOL
        // toward the table and the tool came to rest on top of a box at
        // -0.111 m, so it never got close to anything. What matters is how
        // close the BEAM is to a surface, and the beam is offset 30 mm along
        // x. So the tool is parked beside box 2 with the beam over the box's
        // top face, 13 mm below, which is inside the 20 mm blind zone while
        // the tool itself sits 50 mm clear of the table.
        const arm::IkResult low =
            arm::inverse_kinematics(r.pp.arm, arm::Vec2{0.230, -0.095},
                                    arm::Elbow::kDown);
        if (low.reachable) {
            const u32 weak_before = r.laser.weak_returns();
            r.hold(low.q, 1.5);

            const arm::Vec2 t2 = r.world.tip();
            const double beam_d =
                r.world.distance_below(t2[0] + kLaserOffsetXM, t2[1], r.faults);

            // Assert the arm actually got where the test needs it, so the
            // rejection below is being tested for the right reason.
            std::snprintf(buf, sizeof(buf), "beam %.4f m from the surface, limit %.3f m",
                          beam_d, static_cast<double>(kLaserMinM));
            check("beam is genuinely inside the blind zone",
                  beam_d < static_cast<double>(kLaserMinM), buf);

            std::snprintf(buf, sizeof(buf), "%u weak returns, strength fell to %u",
                          r.laser.weak_returns() - weak_before,
                          r.laser.sample().strength);
            check("a target inside the blind zone is rejected",
                  r.laser.weak_returns() > weak_before &&
                  !r.laser.sample().valid, buf);
        }
        r.hold(q_hold, 1.0);
    }

    // ------------------------------------------- the UART failure that matters
    std::printf("\nrecovering from a corrupted serial line\n");
    {
        const u32 ok_before = r.laser.frames_ok();
        const u32 err_before = r.laser.checksum_errors();

        r.faults.laser_dropouts = true;
        r.hold(q_hold, 2.0);

        const u32 corrupted = r.laser.checksum_errors() - err_before;
        const u32 good = r.laser.frames_ok() - ok_before;

        std::printf("    over 2 s with a noisy line: %u frames good, %u rejected\n",
                    good, corrupted);
        std::snprintf(buf, sizeof(buf), "%u checksum failures caught", corrupted);
        check("corrupted frames are caught by the checksum", corrupted > 3, buf);

        // This is the whole point. A driver that skipped a fixed nine bytes
        // after a checksum failure would be permanently misaligned once a
        // byte went missing, and good frames would stop arriving entirely.
        // Still getting a healthy majority through proves it resynchronises.
        std::snprintf(buf, sizeof(buf), "%u good vs %u bad", good, corrupted);
        check("the link keeps working, so it resynchronised",
              good > corrupted * 2, buf);

        r.faults.laser_dropouts = false;
        r.hold(q_hold, 0.5);
        check("clean line recovers fully", r.laser.sample().valid, "");

        // And silence must be distinguished from a valid stale reading.
        check("not timed out while healthy", !r.laser.timed_out(), "");
        r.faults.laser_silent = true;
        r.hold(q_hold, 0.2);
        std::snprintf(buf, sizeof(buf), "timeout after %.0f ms of silence",
                      kLaserTimeoutUs / 1000.0);
        check("a silent sensor is detected", r.laser.timed_out(), buf);
        r.faults.laser_silent = false;
        r.hold(q_hold, 0.2);
        check("recovers when frames return", !r.laser.timed_out(), "");
    }

    // ------------------------------- the reason there are two position channels
    std::printf("\nthe dual channel catches what neither channel can alone\n");
    {
        // Freeze the SPI encoder on joint 1. It keeps answering, the parity
        // is good, the error flag is clear, and the value is perfectly
        // plausible. Nothing in the encoder driver can detect this.
        r.faults.enc_stuck_joint = 1;
        const arm::Vec2 q_a =
            arm::inverse_kinematics(r.pp.arm, arm::Vec2{0.240, -0.020},
                                    arm::Elbow::kDown).q;
        r.hold(q_a, 1.0);

        const float spi_q1 = joint_angle_from_encoder(r.last_e1, 0);
        const float can_q1 = r.can.drive(0).position_rad;
        const double truth = r.world.q()[0];
        const float disagreement = std::fabs(spi_q1 - can_q1);

        std::printf("    spi encoder  %.4f rad   (frozen)\n", spi_q1);
        std::printf("    can drive    %.4f rad\n", can_q1);
        std::printf("    truth        %.4f rad\n", truth);

        std::snprintf(buf, sizeof(buf), "%u parity, %u error frames",
                      r.enc1.parity_errors(), r.enc1.error_frames());
        check("the stuck encoder still looks perfectly healthy",
              r.last_e1.valid && r.enc1.error_frames() == 0, buf);

        std::snprintf(buf, sizeof(buf), "channels differ by %.4f rad", disagreement);
        check("but the two channels disagree, so it is detectable",
              disagreement > 0.05f, buf);

        std::snprintf(buf, sizeof(buf), "drive within %.4f rad of truth",
                      std::fabs(can_q1 - truth));
        check("the healthy channel still tracks truth",
              std::fabs(can_q1 - truth) < 0.01, buf);
        r.faults.enc_stuck_joint = 0;
        r.hold(q_a, 0.2);
    }

    // ------------------------------------------------------ other faults
    std::printf("\nfaults the drivers can catch on their own\n");
    {
        const u32 ef = r.enc1.error_frames();
        r.faults.enc_error_frames = true;
        r.hold(q_hold, 0.2);
        std::snprintf(buf, sizeof(buf), "%u error frames counted",
                      r.enc1.error_frames() - ef);
        check("encoder error flag rejects the sample",
              r.enc1.error_frames() > ef && !r.last_e1.valid, buf);
        r.faults.enc_error_frames = false;

        const u32 ie = r.imu.error_count();
        r.faults.imu_nack = true;
        r.hold(q_hold, 0.2);
        std::snprintf(buf, sizeof(buf), "%u driver errors", r.imu.error_count() - ie);
        check("imu nack rejects the sample",
              r.imu.error_count() > ie && !r.last_imu.valid, buf);
        r.faults.imu_nack = false;

        r.faults.can_silent_node = kNodeDrive2;
        r.hold(q_hold, 0.5);
        std::snprintf(buf, sizeof(buf), "%u lost events", r.can.node_lost_count());
        check("heartbeat timeout finds the silent drive",
              r.can.node_lost(kNodeDrive2), buf);
        check("the other drive is unaffected", !r.can.node_lost(kNodeDrive1), "");
        r.faults.can_silent_node = 0;
        r.hold(q_hold, 0.4);
        check("node recovers when it speaks again", !r.can.node_lost(kNodeDrive2), "");

        const u32 txe = r.can.tx_errors();
        r.faults.can_bus_off = true;
        r.hold(q_hold, 0.1);
        std::snprintf(buf, sizeof(buf), "%u transmit errors", r.can.tx_errors() - txe);
        check("bus off fails transmits instead of pretending",
              r.can.tx_errors() > txe, buf);
        r.faults.can_bus_off = false;
    }

    std::printf("\n%s  (%d failures)\n",
                g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
    return g_failures == 0 ? 0 : 1;
}
