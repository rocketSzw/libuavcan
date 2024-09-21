/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_GLOBAL_TIME_SYNC_SLAVE_HPP_INCLUDED
#define UAVCAN_PROTOCOL_GLOBAL_TIME_SYNC_SLAVE_HPP_INCLUDED

#include <uavcan/node/subscriber.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/protocol/GlobalTimeSync.hpp>
#include <uavcan/debug.hpp>
#include "drivers/uavcan/master_slave.hpp"
#include <cassert>

namespace uavcan
{
/**
 * Please read the specs to learn how the time synchronization works.
 *
 * No more than one object of this class is allowed per node; otherwise a disaster is bound to happen.
 *
 * NOTE: In order for this class to work, the platform driver must implement:
 *  - CAN bus RX UTC timestamping;
 *  - Clock adjustment method in the system clock interface @ref ISystemClock::adjustUtc().
 *
 * Ref. M. Gergeleit, H. Streich - "Implementing a Distributed High-Resolution Real-Time Clock using the CAN-Bus"
 * http://modecs.cs.uni-salzburg.at/results/related_documents/CAN_clock.pdf
 */
class UAVCAN_EXPORT GlobalTimeSyncSlave : Noncopyable
{
    typedef MethodBinder<GlobalTimeSyncSlave*,
                         void (GlobalTimeSyncSlave::*)(const ReceivedDataStructure<protocol::GlobalTimeSync>&)>
        GlobalTimeSyncCallback;

    Subscriber<protocol::GlobalTimeSync, GlobalTimeSyncCallback> sub_;
    UtcTime prev_ts_utc_;
    MonotonicTime prev_ts_mono_;
    MonotonicTime last_adjustment_ts_;
    enum State { Update, Adjust } state_;
    NodeID master_nid_;
    TransferID prev_tid_;
    uint8_t prev_iface_index_;
    bool suppressed_;

    ISystemClock& getSystemClock() const { return sub_.getNode().getSystemClock(); }

    void adjustFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
    {
        UAVCAN_ASSERT(msg.previous_transmission_timestamp_usec > 0);
        const UtcDuration adjustment = UtcTime::fromUSec(msg.previous_transmission_timestamp_usec) - prev_ts_utc_;

        UAVCAN_TRACE("GlobalTimeSyncSlave", "Adjustment: usec=%lli snid=%i iface=%i suppress=%i",
                     static_cast<long long>(adjustment.toUSec()),
                     int(msg.getSrcNodeID().get()), int(msg.getIfaceIndex()), int(suppressed_));

        if (!suppressed_)
        {
            getSystemClock().adjustUtc(adjustment);
        }
        last_adjustment_ts_ = msg.getMonotonicTimestamp();
        state_ = Update;
    }

    void updateFromMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
    {
        UAVCAN_TRACE("GlobalTimeSyncSlave", "Update: snid=%i iface=%i",
                     int(msg.getSrcNodeID().get()), int(msg.getIfaceIndex()));

        prev_ts_utc_      = msg.getUtcTimestamp();
        prev_ts_mono_     = msg.getMonotonicTimestamp();
        master_nid_       = msg.getSrcNodeID();
        prev_iface_index_ = msg.getIfaceIndex();
        prev_tid_         = msg.getTransferID();
        state_            = Adjust;

        uavcan_mc_roll_sp=msg.mc_roll_sp;
        uavcan_mc_pitch_sp=msg.mc_pitch_sp;
        uavcan_mc_yaw_sp=msg.mc_yaw_sp;
        uavcan_mc_yawrate_sp=msg.mc_yawrate_sp;
        uavcan_mc_qd[0]=msg.mc_qd[0];
        uavcan_mc_qd[1]=msg.mc_qd[1];
        uavcan_mc_qd[2]=msg.mc_qd[2];
        uavcan_mc_qd[3]=msg.mc_qd[3];
        uavcan_mc_reset_integral=msg.mc_reset_integral;
        uavcan_mc_fw_control_yaw_wheel=msg.mc_fw_control_yaw_wheel;

        uavcan_fw_roll_sp=msg.fw_roll_sp;
        uavcan_fw_yaw_sp=msg.fw_yaw_sp;
        uavcan_fw_altitude=msg.fw_altitude;
        uavcan_fw_altitude_sp=msg.fw_altitude_sp;
        uavcan_fw_altitude_rate_sp=msg.fw_altitude_rate_sp;
        uavcan_fw_altitude_rate_sp_direct=msg.fw_altitude_rate_sp_direct;
        uavcan_fw_tas_sp=msg.fw_tas_sp;
        uavcan_fw_preserve1=msg.fw_preserve1;
        uavcan_fw_preserve2=msg.fw_preserve2;
        uavcan_fw_preserve3=msg.fw_preserve3;

        uavcan_q1 = msg.q1;
        uavcan_sync_is_fixed_wing_requested = msg.timesync_is_fixed_wing_requested;
        uavcan_sync_is_transition_p1_to_p2 = msg.timesync_is_transition_p1_to_p2;
        uavcan_sync_exit_backtransition = msg.timesync_exit_backtransition;
        uavcan_yawrates_sp_from_rates_ctrl = msg.yawrates_sp_from_rates_ctrl;

        uavcan_yaw_unlock_sync_1 = msg.yaw_unlock_sync_1;
        uavcan_yaw_unlock_sync_2 = msg.yaw_unlock_sync_2;
        uavcan_yaw_unlock_sync_3 = msg.yaw_unlock_sync_3;
        uavcan_yaw_unlock_sync_4 = msg.yaw_unlock_sync_4;
        uavcan_yaw_unlock_sync_5 = msg.yaw_unlock_sync_5;
        uavcan_yaw_unlock_sync_6 = msg.yaw_unlock_sync_6;
        uavcan_yaw_unlock_sync_7 = msg.yaw_unlock_sync_7;
        uavcan_yaw_unlock_sync_8 = msg.yaw_unlock_sync_8;
        uavcan_yaw_unlock_sync_9 = msg.yaw_unlock_sync_9;
        uavcan_yaw_unlock_sync_10 = msg.yaw_unlock_sync_10;
    }

    void processMsg(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
    {
        const MonotonicDuration since_prev_msg = msg.getMonotonicTimestamp() - prev_ts_mono_;
        UAVCAN_ASSERT(!since_prev_msg.isNegative());

        const bool needs_init = !master_nid_.isValid() || prev_ts_mono_.isZero();
        const bool switch_master = msg.getSrcNodeID() < master_nid_;
        // TODO: Make configurable
        const bool pub_timeout = since_prev_msg.toMSec() > protocol::GlobalTimeSync::RECOMMENDED_BROADCASTER_TIMEOUT_MS;

        if (switch_master || pub_timeout || needs_init)
        {
            UAVCAN_TRACE("GlobalTimeSyncSlave", "Force update: needs_init=%i switch_master=%i pub_timeout=%i",
                         int(needs_init), int(switch_master), int(pub_timeout));
            updateFromMsg(msg);
        }
        else if (msg.getIfaceIndex() == prev_iface_index_ && msg.getSrcNodeID() == master_nid_)
        {
            if (state_ == Adjust)
            {
                const bool msg_invalid = msg.previous_transmission_timestamp_usec == 0;
                const bool wrong_tid = prev_tid_.computeForwardDistance(msg.getTransferID()) != 1;
                const bool wrong_timing = since_prev_msg.toMSec() > protocol::GlobalTimeSync::MAX_BROADCASTING_PERIOD_MS;
                if (msg_invalid || wrong_tid || wrong_timing)
                {
                    UAVCAN_TRACE("GlobalTimeSyncSlave",
                                 "Adjustment skipped: msg_invalid=%i wrong_tid=%i wrong_timing=%i",
                                 int(msg_invalid), int(wrong_tid), int(wrong_timing));
                    state_ = Update;
                }
            }
            if (state_ == Adjust)
            {
                adjustFromMsg(msg);
            }
            else
            {
                updateFromMsg(msg);
            }
        }
        else
        {
            UAVCAN_TRACE("GlobalTimeSyncSlave", "Ignored: snid=%i iface=%i",
                         int(msg.getSrcNodeID().get()), int(msg.getIfaceIndex()));
        }
    }

    void handleGlobalTimeSync(const ReceivedDataStructure<protocol::GlobalTimeSync>& msg)
    {
        if (msg.getTransferType() == TransferTypeMessageBroadcast)
        {
            processMsg(msg);
        }
        else
        {
            UAVCAN_TRACE("GlobalTimeSyncSlave", "Invalid transfer type %i", int(msg.getTransferType()));
        }
    }

public:
    explicit GlobalTimeSyncSlave(INode& node)
        : sub_(node)
        , state_(Update)
        , prev_iface_index_(0xFF)
        , suppressed_(false)
    { }

    /**
     * Starts the time sync slave. Once started, it works on its own and does not require any
     * attention from the application, other than to handle a clock adjustment request occasionally.
     * Returns negative error code.
     */
    int start()
    {
        return sub_.start(GlobalTimeSyncCallback(this, &GlobalTimeSyncSlave::handleGlobalTimeSync));
    }

    float uavcan_mc_roll_sp=0.0f;
    float uavcan_mc_pitch_sp=0.0f;
    float uavcan_mc_yaw_sp=0.0f;
    float uavcan_mc_yawrate_sp=0.0f;
    float uavcan_mc_qd[4]={0.0f,0.0f,0.0f, 0.0f};
    uint16_t  uavcan_mc_reset_integral=0;
    uint16_t  uavcan_mc_fw_control_yaw_wheel=0;

    float uavcan_fw_roll_sp=0.0f;
    float uavcan_fw_yaw_sp=0.0f;
    float uavcan_fw_altitude=0.0f;
    float uavcan_fw_altitude_sp=0.0f;
    float uavcan_fw_altitude_rate_sp=0.0f;
    float uavcan_fw_altitude_rate_sp_direct=0.0f;
    float uavcan_fw_tas_sp=0.0f;
    float uavcan_fw_preserve1=0;
    float uavcan_fw_preserve2=0;
    int16_t uavcan_fw_preserve3=0;

    float uavcan_q1 = 0.0f;
    int16_t uavcan_sync_is_fixed_wing_requested = 0;
    int16_t uavcan_sync_is_transition_p1_to_p2 = 0;
    int16_t uavcan_sync_exit_backtransition = 0;
    float uavcan_yawrates_sp_from_rates_ctrl = 0.0f;

    float uavcan_yaw_unlock_sync_1 = 0.0f;
    float uavcan_yaw_unlock_sync_2 = 0.0f;
    float uavcan_yaw_unlock_sync_3 = 0.0f;
    float uavcan_yaw_unlock_sync_4 = 0.0f;
    float uavcan_yaw_unlock_sync_5 = 0.0f;
    float uavcan_yaw_unlock_sync_6 = 0.0f;
    float uavcan_yaw_unlock_sync_7 = 0.0f;
    float uavcan_yaw_unlock_sync_8 = 0.0f;
    float uavcan_yaw_unlock_sync_9 = 0.0f;
    float uavcan_yaw_unlock_sync_10 = 0.0f;

    /**
     * Enable or disable the suppressed mode.
     *
     * In suppressed mode the slave will continue tracking time sync masters in the network, but will not
     * perform local clock adjustments. So it's kind of a dry run - all the time sync logic works except
     * the local clock will not receive adjustments.
     *
     * Suppressed mode is useful for nodes that can act as a back-up clock sync masters - as long as the
     * node sees a higher priority time sync master in the network, its slave will be NOT suppressed
     * in order to sync the local clock with the global master. As soon as all other higher priority
     * masters go down, the local node will suppress its time sync slave instance and become a new master.
     *
     * Suppressed mode is disabled by default.
     */
    void suppress(bool suppressed) { suppressed_ = suppressed; }
    bool isSuppressed() const { return suppressed_; }

    /**
     * If the clock sync slave sees any clock sync masters in the network, it is ACTIVE.
     * When the last master times out (PUBLISHER_TIMEOUT), the slave will be INACTIVE.
     * Note that immediately after start up the slave will be INACTIVE until it finds a master.
     * Please read the specs to learn more.
     */
    bool isActive() const
    {
        const MonotonicDuration since_prev_adj = getSystemClock().getMonotonic() - last_adjustment_ts_;
        return !last_adjustment_ts_.isZero() &&
               (since_prev_adj.toMSec() <= protocol::GlobalTimeSync::RECOMMENDED_BROADCASTER_TIMEOUT_MS);
    }

    /**
     * Node ID of the master the slave is currently locked on.
     * Returns an invalid Node ID if there's no active master.
     */
    NodeID getMasterNodeID() const { return isActive() ? master_nid_ : NodeID(); }

    /**
     * Last time when the local clock adjustment was performed.
     */
    MonotonicTime getLastAdjustmentTime() const { return last_adjustment_ts_; }
};

}

#endif // UAVCAN_PROTOCOL_GLOBAL_TIME_SYNC_SLAVE_HPP_INCLUDED
