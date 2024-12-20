/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_PROTOCOL_GLOBAL_TIME_SYNC_MASTER_HPP_INCLUDED
#define UAVCAN_PROTOCOL_GLOBAL_TIME_SYNC_MASTER_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/publisher.hpp>
#include <uavcan/node/subscriber.hpp>
#include <uavcan/util/method_binder.hpp>
#include <uavcan/util/lazy_constructor.hpp>
#include <uavcan/protocol/GlobalTimeSync.hpp>
#include <uavcan/debug.hpp>
#include <cstdlib>
#include <cassert>
#include <uORB/Subscription.hpp>

namespace uavcan
{
/**
 * Please read the specs to learn how the time synchronization works.
 *
 * No more than one object of this class is allowed per node; otherwise a disaster is bound to happen.
 *
 * NOTE: In order for this class to work, the platform driver must implement
 *       CAN bus TX loopback with both UTC and monotonic timestamping.
 *
 * Ref. M. Gergeleit, H. Streich - "Implementing a Distributed High-Resolution Real-Time Clock using the CAN-Bus"
 *
 * TODO: Enforce max one master per node
 */
class UAVCAN_EXPORT GlobalTimeSyncMaster : protected LoopbackFrameListenerBase
{
    class IfaceMaster
    {
        Publisher<protocol::GlobalTimeSync> pub_;
        MonotonicTime iface_prev_pub_mono_;
        UtcTime prev_tx_utc_;
        const uint8_t iface_index_;

    public:
        IfaceMaster(INode& node, uint8_t iface_index)
            : pub_(node)
            , iface_index_(iface_index)
        {
            UAVCAN_ASSERT(iface_index < MaxCanIfaces);
        }

        int init(TransferPriority priority)
        {
            const int res = pub_.init(priority);
            if (res >= 0)
            {
                pub_.getTransferSender().setIfaceMask(uint8_t(1 << iface_index_));
                pub_.getTransferSender().setCanIOFlags(CanIOFlagLoopback);
            }
            return res;
        }

        void setTxTimestamp(UtcTime ts)
        {
            if (ts.isZero())
            {
                UAVCAN_ASSERT(0);
                pub_.getNode().registerInternalFailure("GlobalTimeSyncMaster zero TX ts");
                return;
            }
            if (!prev_tx_utc_.isZero())
            {
                prev_tx_utc_ = UtcTime(); // Reset again, because there's something broken in the driver and we don't trust it
                pub_.getNode().registerInternalFailure("GlobalTimeSyncMaster pub conflict");
                return;
            }
            prev_tx_utc_ = ts;
        }

        int publish(TransferID tid, MonotonicTime current_time, float uavcaninfo[])
        {
            UAVCAN_ASSERT(pub_.getTransferSender().getCanIOFlags() == CanIOFlagLoopback);
            UAVCAN_ASSERT(pub_.getTransferSender().getIfaceMask() == (1 << iface_index_));

            const MonotonicDuration since_prev_pub = current_time - iface_prev_pub_mono_;
            iface_prev_pub_mono_ = current_time;
            UAVCAN_ASSERT(since_prev_pub.isPositive());
            const bool long_period = since_prev_pub.toMSec() >= protocol::GlobalTimeSync::MAX_BROADCASTING_PERIOD_MS;

            protocol::GlobalTimeSync msg;
            msg.previous_transmission_timestamp_usec = long_period ? 0 : prev_tx_utc_.toUSec();
            msg.mc_roll_sp = uavcaninfo[0];
            msg.mc_pitch_sp = uavcaninfo[1];
            msg.mc_yaw_sp = uavcaninfo[2];
            msg.mc_yawrate_sp = uavcaninfo[3];
            msg.mc_qd[0] = uavcaninfo[4];
            msg.mc_qd[1] = uavcaninfo[5];
            msg.mc_qd[2] = uavcaninfo[6];
            msg.mc_qd[3] = uavcaninfo[7];
            msg.mc_reset_integral = (uint16_t)uavcaninfo[8];
            msg.mc_fw_control_yaw_wheel = (uint16_t)uavcaninfo[9];

            msg.fw_roll_sp = uavcaninfo[10];
            msg.fw_yaw_sp = uavcaninfo[11];
            msg.fw_altitude = uavcaninfo[12];
            msg.fw_altitude_sp = uavcaninfo[13];
            msg.fw_altitude_rate_sp = uavcaninfo[14];
            msg.fw_altitude_rate_sp_direct = uavcaninfo[15];
            msg.fw_tas_sp = uavcaninfo[16];
            msg.fw_preserve1 = uavcaninfo[17];
            msg.fw_preserve2 = uavcaninfo[18];
            msg.fw_preserve3 = uavcaninfo[19];

            msg.timesync_is_fixed_wing_requested = (int16_t)uavcaninfo[20];
            msg.timesync_is_transition_p1_to_p2 = (int16_t)uavcaninfo[21];
            msg.timesync_exit_backtransition = (int16_t)uavcaninfo[22];
            msg.q1 = uavcaninfo[23];
            msg.yawrates_sp_from_rates_ctrl = uavcaninfo[24];

            msg.yaw_unlock_sync_1 = uavcaninfo[25];
            msg.yaw_unlock_sync_2 = uavcaninfo[26];
            msg.yaw_unlock_sync_3 = uavcaninfo[27];
            msg.yaw_unlock_sync_4 = uavcaninfo[28];
            msg.yaw_unlock_sync_5 = uavcaninfo[29];
            msg.yaw_unlock_sync_6 = uavcaninfo[30];
            msg.yaw_unlock_sync_7 = uavcaninfo[31];
            msg.yaw_unlock_sync_8 = uavcaninfo[32];
            msg.yaw_unlock_sync_9 = uavcaninfo[33];
            msg.yaw_unlock_sync_10 = uavcaninfo[34];

            prev_tx_utc_ = UtcTime();

            UAVCAN_TRACE("GlobalTimeSyncMaster", "Publishing %llu iface=%i tid=%i",
                        static_cast<unsigned long long>(msg.previous_transmission_timestamp_usec),
                        int(iface_index_), int(tid.get()));

            return pub_.broadcast(msg, tid);
        }
    };

    INode& node_;
    LazyConstructor<IfaceMaster> iface_masters_[MaxCanIfaces];
    MonotonicTime prev_pub_mono_;
    DataTypeID dtid_;
    bool initialized_;

    virtual void handleLoopbackFrame(const RxFrame& frame)
    {
        const uint8_t iface = frame.getIfaceIndex();
        if (initialized_ && iface < MaxCanIfaces)
        {
            if (frame.getDataTypeID() == dtid_ &&
                frame.getTransferType() == TransferTypeMessageBroadcast &&
                frame.isStartOfTransfer() && frame.isEndOfTransfer() &&
                frame.getSrcNodeID() == node_.getNodeID())
            {
                iface_masters_[iface]->setTxTimestamp(frame.getUtcTimestamp());
            }
        }
        else
        {
            UAVCAN_ASSERT(0);
        }
    }

    int getNextTransferID(TransferID& tid)
    {
        const MonotonicDuration max_transfer_interval =
            MonotonicDuration::fromMSec(protocol::GlobalTimeSync::MAX_BROADCASTING_PERIOD_MS);

        const OutgoingTransferRegistryKey otr_key(dtid_, TransferTypeMessageBroadcast, NodeID::Broadcast);
        const MonotonicTime otr_deadline = node_.getMonotonicTime() + max_transfer_interval;
        TransferID* const tid_ptr =
            node_.getDispatcher().getOutgoingTransferRegistry().accessOrCreate(otr_key, otr_deadline);
        if (tid_ptr == UAVCAN_NULLPTR)
        {
            return -ErrMemory;
        }

        tid = *tid_ptr;
        tid_ptr->increment();
        return 0;
    }

public:
    explicit GlobalTimeSyncMaster(INode& node)
        : LoopbackFrameListenerBase(node.getDispatcher())
        , node_(node)
        , initialized_(false)
    { }

    /**
     * Merely prepares the class to work, doesn't do anything else.
     * Must be called before the master can be used.
     * Returns negative error code.
     */
    int init(const TransferPriority priority = TransferPriority::OneLowerThanHighest)
    {
        if (initialized_)
        {
            return 0;
        }

        // Data type ID
        const DataTypeDescriptor* const desc =
            GlobalDataTypeRegistry::instance().find(DataTypeKindMessage, protocol::GlobalTimeSync::getDataTypeFullName());
        if (desc == UAVCAN_NULLPTR)
        {
            return -ErrUnknownDataType;
        }
        dtid_ = desc->getID();

        // Iface master array
        int res = -ErrLogic;
        for (uint8_t i = 0; i < MaxCanIfaces; i++)
        {
            if (!iface_masters_[i].isConstructed())
            {
                iface_masters_[i].construct<INode&, uint8_t>(node_, i);
            }
            res = iface_masters_[i]->init(priority);
            if (res < 0)
            {
                break;
            }
        }

        // Loopback listener
        initialized_ = res >= 0;
        if (initialized_)
        {
            LoopbackFrameListenerBase::startListening();
        }
        return res;
    }

    /**
     * Whether the master instance has been initialized.
     */
    bool isInitialized() const { return initialized_; }

    /**
     * Publishes one sync message.
     *
     * Every call to this method hints the master to publish the next sync message once. Exact time will
     * be obtained from the TX loopback timestamp field.
     *
     * This method shall be called with a proper interval - refer to the time sync message definition
     * for min/max interval values.
     *
     * Returns negative error code.
     */
    int publish(float uavcaninfo[])
    {
        if (!initialized_)
        {
            const int res = init();
            if (res < 0)
            {
                return res;
            }
        }

        /*
         * Enforce max frequency
         */
        const MonotonicTime current_time = node_.getMonotonicTime();
        {
            const MonotonicDuration since_prev_pub = current_time - prev_pub_mono_;
            UAVCAN_ASSERT(since_prev_pub.isPositive());
            if (since_prev_pub.toMSec() < protocol::GlobalTimeSync::MIN_BROADCASTING_PERIOD_MS)
            {
                UAVCAN_TRACE("GlobalTimeSyncMaster", "Publication skipped");
                return 0;
            }
            prev_pub_mono_ = current_time;
        }

        /*
         * Obtain common Transfer ID for all masters
         */
        TransferID tid;
        {
            const int tid_res = getNextTransferID(tid);
            if (tid_res < 0)
            {
                return tid_res;
            }
        }

        for (uint8_t i = 0; i < node_.getDispatcher().getCanIOManager().getNumIfaces(); i++)
        {
            const int res = iface_masters_[i]->publish(tid, current_time, &uavcaninfo[0]);
            if (res < 0)
            {
                return res;
            }
        }
        return 0;
    }
};

}

#endif // UAVCAN_PROTOCOL_GLOBAL_TIME_SYNC_MASTER_HPP_INCLUDED
