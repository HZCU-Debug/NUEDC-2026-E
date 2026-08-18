#include <cassert>
#include <vector>

#include "comm/link.h"

int main() {
    HardwareSerial serial;
    comm::Link<8> link(serial, comm::LinkConfig(115200, 16, 17, 50));

    assert(link.begin());

    assert(link.send(0, NULL, 0, comm::Delivery::Unreliable) ==
           comm::SendResult::InvalidArgument);
    assert(link.send(1, NULL, 1, comm::Delivery::Unreliable) ==
           comm::SendResult::InvalidArgument);
    assert(link.send(1, NULL, 0, static_cast<comm::Delivery>(2)) ==
           comm::SendResult::InvalidArgument);
    const uint8_t oversized[9] = {};
    assert(link.send(1, oversized, sizeof(oversized), comm::Delivery::Unreliable) ==
           comm::SendResult::PayloadTooLarge);

    const uint8_t payload[] = {0x11, 0x00, 0x22};
    assert(link.send(1, payload, sizeof(payload), comm::Delivery::Unreliable) ==
           comm::SendResult::Accepted);
    assert(serial.transmitted ==
           std::vector<uint8_t>({0x03, 0x01, 0x11, 0x03, 0x22, 0x31, 0x00}));

    serial.transmitted.clear();
    const uint8_t reliablePayload[] = {0x33};
    assert(link.send(1, reliablePayload, sizeof(reliablePayload),
                     comm::Delivery::Reliable) == comm::SendResult::Accepted);
    assert(serial.transmitted ==
           std::vector<uint8_t>({0x02, 0x81, 0x03, 0x33, 0xF9, 0x00}));

    assert(link.send(2, NULL, 0, comm::Delivery::Unreliable) ==
           comm::SendResult::Busy);
    assert(link.send(2, NULL, 0, comm::Delivery::Reliable) ==
           comm::SendResult::Busy);
    assert(serial.transmitted.size() == 6);

    serial.receive({0x01, 0x01, 0x01, 0x01, 0x00});
    assert(link.poll().type == comm::EventType::Delivered);
    assert(link.poll().type == comm::EventType::None);
    assert(link.send(2, NULL, 0, comm::Delivery::Unreliable) ==
           comm::SendResult::Accepted);

    serial.receive({0x03, 0x01, 0x11, 0x03, 0x22, 0x31, 0x00});
    const comm::Event unreliable = link.poll();
    assert(unreliable.type == comm::EventType::Message);
    assert(unreliable.message.type == 1);
    assert(unreliable.message.delivery == comm::Delivery::Unreliable);
    assert(unreliable.message.size == 3);
    assert(unreliable.message.payload[0] == 0x11);
    assert(unreliable.message.payload[1] == 0x00);
    assert(unreliable.message.payload[2] == 0x22);

    serial.transmitted.clear();
    serial.receive({0x05, 0x81, 0x07, 0x44, 0xD0, 0x00});
    const comm::Event reliable = link.poll();
    assert(reliable.type == comm::EventType::Message);
    assert(reliable.message.type == 1);
    assert(reliable.message.delivery == comm::Delivery::Reliable);
    assert(reliable.message.size == 1);
    assert(reliable.message.payload[0] == 0x44);
    assert(serial.transmitted ==
           std::vector<uint8_t>({0x01, 0x03, 0x07, 0x15, 0x00}));

    serial.receive({0x05, 0x81, 0x07, 0x44, 0xD0, 0x00});
    assert(link.poll().type == comm::EventType::None);
    assert(serial.transmitted == std::vector<uint8_t>({
                                     0x01, 0x03, 0x07, 0x15, 0x00,
                                     0x01, 0x03, 0x07, 0x15, 0x00,
                                 }));

    HardwareSerial retrySerial;
    comm::Link<8> retryLink(retrySerial, comm::LinkConfig(115200, 16, 17, 1));
    assert(retryLink.begin());
    assert(retryLink.send(1, reliablePayload, sizeof(reliablePayload),
                          comm::Delivery::Reliable) == comm::SendResult::Accepted);
    const std::vector<uint8_t> firstAttempt = retrySerial.transmitted;
    assert(retryLink.poll().type == comm::EventType::None);
    std::vector<uint8_t> repeated = firstAttempt;
    repeated.insert(repeated.end(), firstAttempt.begin(), firstAttempt.end());
    assert(retrySerial.transmitted == repeated);

    retryLink.cancel();
    assert(retryLink.send(2, NULL, 0, comm::Delivery::Unreliable) ==
           comm::SendResult::Accepted);

    HardwareSerial recoverySerial;
    comm::Link<8> recoveryLink(recoverySerial);
    assert(recoveryLink.begin());
    recoverySerial.receive({
        0x03, 0x01, 0x11, 0x03, 0x22, 0x30, 0x00,
        0x05, 0x01, 0x00,
        0x03, 0x01, 0x11, 0x03, 0x22, 0x31, 0x00,
    });
    assert(recoveryLink.poll().type == comm::EventType::Message);

    recoverySerial.receive({0x03, 0x01, 0x11});
    assert(recoveryLink.poll().type == comm::EventType::None);
    recoverySerial.receive({0x03, 0x22, 0x31, 0x00});
    assert(recoveryLink.poll().type == comm::EventType::Message);

    HardwareSerial crcSerial;
    comm::Link<8> crcLink(crcSerial);
    assert(crcLink.begin());
    const uint8_t crcPayload[] = {'2', '3', '4', '5', '6', '7', '8', '9'};
    assert(crcLink.send('1', crcPayload, sizeof(crcPayload),
                        comm::Delivery::Unreliable) == comm::SendResult::Accepted);
    assert(crcSerial.transmitted == std::vector<uint8_t>({
                                        0x0B, '1', '2', '3', '4', '5',
                                        '6',  '7', '8', '9', 0xF4, 0x00,
                                    }));
}
