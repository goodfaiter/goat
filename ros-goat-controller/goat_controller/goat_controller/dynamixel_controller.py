import sys
from dynamixel_sdk import *
from .dynamixel_address_book import *
import ctypes


def uint16_to_int16(value):
    """Convert unsigned 16-bit to signed 16-bit"""
    return ctypes.c_int16(value).value

def uint32_to_int32(value):
    """Convert unsigned 32-bit to signed 32-bit"""
    return ctypes.c_int32(value).value


class Dynamixel:
    def __init__(self, ID, descriptive_device_name, port_name, baudrate, series_name="xm"):
        # Communication inputs
        if type(ID) == list:
            self.multiple_motors = True
        else:
            self.multiple_motors = False

        self.ID = ID
        self.descriptive_device_name = descriptive_device_name
        self.port_name = port_name
        self.baudrate = baudrate

        # Set series name
        if type(self.ID) == list:
            if type(series_name) == list and len(series_name) == len(self.ID):
                self.series_name = {}
                for i in range(len(self.ID)):
                    self.series_name[self.ID[i]] = series_name[i]
            else:
                print("Provide correct series name type / length")
                sys.exit()

        else:
            if type(series_name) == str:
                self.series_name = {self.ID: series_name}
            else:
                print("Provide correct series name type")

        for id, series in self.series_name.items():
            # Check for series name
            all_series_names = ["xm", "xl", "xw", "xc"]
            if series not in all_series_names:
                print("Series name invalid for motor with ID,", id, "Choose one of:", all_series_names)
                sys.exit()

        # Communication settings
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Setup group read/writes
        self.groupSyncWritePosition = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_POSITION, LEN_POSITION)
        self.groupSyncWriteVelocity = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_VELOCITY, LEN_VELOCITY)
        self.groupSyncWriteCurrent = GroupSyncWrite(self.port_handler, self.packet_handler, ADDR_GOAL_CURRENT, LEN_CURRENT)
        self.groupSyncReadPosition = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_PRESENT_POSITION, LEN_POSITION)
        self.groupSyncReadVelocity = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
        self.groupSyncReadCurrent = GroupSyncRead(self.port_handler, self.packet_handler, ADDR_PRESENT_CURRENT, LEN_CURRENT)

    def fetch_and_check_ID(self, ID):
        if self.multiple_motors:
            if ID is None:
                print("You specified multiple dynamixels on this port. But did not specify which motor to operate upon. Please specify ID.")
                sys.exit()
            elif ID == "all":
                return self.ID
            elif type(ID) == list:
                for id in ID:
                    if id not in self.ID:
                        print("The ID you specified:", id, "in the list", ID, "does not exist in the list of IDs you initialized.")
                        sys.exit()
                return ID
            else:
                if ID in self.ID:
                    return [ID]
                else:
                    print("The ID you specified:", ID, "does not exist in the list of IDs you initialized.")
                    sys.exit()
        else:
            return [self.ID]

    def begin_communication(self):
        # Open port
        try:
            self.port_handler.openPort()
            print("Port open successfully for:", self.descriptive_device_name)
        except:
            print("!! Failed to open port for:", self.descriptive_device_name)
            print("Check: \n1. If correct port name is specified\n2. If dynamixel wizard isn't connected")
            sys.exit()

        # Set port baudrate
        try:
            self.port_handler.setBaudRate(self.baudrate)
            print("Baudrate set successfully for:", self.descriptive_device_name)
        except:
            print("!! Failed to set baudrate for:", self.descriptive_device_name)
            sys.exit()

    def end_communication(self):
        # Close port
        try:
            self.port_handler.closePort()
            print("Port closed successfully for:", self.descriptive_device_name)
        except:
            print("!! Failed to close port for:", self.descriptive_device_name)
            sys.exit()

    def _print_error_msg(self, process_name, dxl_comm_result, dxl_error, selected_ID, print_only_if_error=False):
        if dxl_comm_result != COMM_SUCCESS:
            print("!!", process_name, "failed for:", self.descriptive_device_name)
            print("Communication error:", self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("!!", process_name, "failed for:", self.descriptive_device_name)
            print("Dynamixel error:", self.packet_handler.getRxPacketError(dxl_error))
        else:
            if not print_only_if_error:
                print(process_name, "successful for:", self.descriptive_device_name, "ID:", selected_ID)

    def enable_torque(self, print_only_if_error=False, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)

        for selected_ID in selected_IDs:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, selected_ID, ADDR_TORQUE_ENABLE, 1)
            self._print_error_msg(
                "Torque enable",
                dxl_comm_result=dxl_comm_result,
                dxl_error=dxl_error,
                selected_ID=selected_ID,
                print_only_if_error=print_only_if_error,
            )

    def disable_torque(self, print_only_if_error=False, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        for selected_ID in selected_IDs:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, selected_ID, ADDR_TORQUE_ENABLE, 0)
            self._print_error_msg(
                "Torque disable",
                dxl_comm_result=dxl_comm_result,
                dxl_error=dxl_error,
                selected_ID=selected_ID,
                print_only_if_error=print_only_if_error,
            )

    def is_torque_on(self, print_only_if_error=False, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        for selected_ID in selected_IDs:
            torque_status, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
                self.port_handler, selected_ID, ADDR_TORQUE_ENABLE
            )
            self._print_error_msg(
                "Read torque status",
                dxl_comm_result=dxl_comm_result,
                dxl_error=dxl_error,
                selected_ID=selected_ID,
                print_only_if_error=print_only_if_error,
            )

            if torque_status == False:
                return False

        return True

    def ping(self, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        for selected_ID in selected_IDs:
            _, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, selected_ID)
            self._print_error_msg("Ping", dxl_comm_result=dxl_comm_result, dxl_error=dxl_error, selected_ID=selected_ID)

    def set_current_limit(self, max_current_ma, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        for selected_ID in selected_IDs:
            int_max_current_ma = min(1193, int(max_current_ma / 2.69))  # one increment being 2.69 mA. 1193 is max range
            self.packet_handler.write2ByteTxRx(self.port_handler, selected_ID, ADDR_CURRENT_LIMIT, int_max_current_ma)

    def set_velocity_pid(self, p: int, i: int, d: int, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        for selected_ID in selected_IDs:

            was_torque_on = False
            if self.is_torque_on(print_only_if_error=True, ID=selected_ID):
                was_torque_on = True
                self.disable_torque(print_only_if_error=True, ID=selected_ID)

            # Write gains (2-byte writes)
            self.packet_handler.write2ByteTxRx(self.port_handler, selected_ID, ADDR_VELOCITY_P_GAIN, p)
            self.packet_handler.write2ByteTxRx(self.port_handler, selected_ID, ADDR_VELOCITY_I_GAIN, i)
            self.packet_handler.write2ByteTxRx(self.port_handler, selected_ID, ADDR_VELOCITY_D_GAIN, d)

            if was_torque_on:
                self.enable_torque(print_only_if_error=True, ID=selected_ID)

    def set_operating_mode(self, mode, ID=None, print_only_if_error=False):
        selected_IDs = self.fetch_and_check_ID(ID)
        for selected_ID in selected_IDs:

            series = self.series_name[selected_ID]
            if series == "xl":
                operating_modes = operating_modes_xl
            else:
                operating_modes = operating_modes_xm

            if mode in operating_modes:
                # Check if torque was enabled
                was_torque_on = False
                if self.is_torque_on(print_only_if_error=True, ID=selected_ID):
                    was_torque_on = True
                    self.disable_torque(print_only_if_error=True, ID=selected_ID)

                mode_id = operating_modes[mode]
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                    self.port_handler, selected_ID, ADDR_OPERATING_MODE, mode_id
                )
                self._print_error_msg(
                    "Mode set to " + mode + " control",
                    dxl_comm_result=dxl_comm_result,
                    dxl_error=dxl_error,
                    selected_ID=selected_ID,
                    print_only_if_error=print_only_if_error,
                )

                if was_torque_on:
                    self.enable_torque(print_only_if_error=True, ID=selected_ID)
            else:
                print("Enter valid operating mode. Select one of:\n" + str(list(operating_modes.keys())))

    def receive_group_sync(self, group_sync: GroupSyncRead) -> None:
        dxl_comm_result = group_sync.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self._print_error_msg(
                "Receive group sync", dxl_comm_result=dxl_comm_result, dxl_error="", selected_ID="", print_only_if_error=True
            )

    def read_from_group_sync(self, ids, group_sync: GroupSyncRead, address, len) -> list:
        vals = []
        for id in ids:
            if group_sync.isAvailable(id, address, len):
                if len == 2:
                    vals.append(uint16_to_int16(group_sync.getData(id, address, len)))
                elif len == 4:
                    vals.append(uint32_to_int32(group_sync.getData(id, address, len)))
        return vals

    def read_position(self, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        for id in selected_IDs:
            self.groupSyncReadPosition.addParam(id)
        self.receive_group_sync(self.groupSyncReadPosition)
        pos = self.read_from_group_sync(selected_IDs, self.groupSyncReadPosition, ADDR_PRESENT_POSITION, LEN_POSITION)
        self.groupSyncReadPosition.clearParam()
        return pos

    def read_velocity(self, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        for id in selected_IDs:
            self.groupSyncReadVelocity.addParam(id)
        self.receive_group_sync(self.groupSyncReadVelocity)
        vel = self.read_from_group_sync(selected_IDs, self.groupSyncReadVelocity, ADDR_PRESENT_VELOCITY, LEN_VELOCITY)
        self.groupSyncReadVelocity.clearParam()
        return vel

    def read_current(self, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        for id in selected_IDs:
            self.groupSyncReadCurrent.addParam(id)
        self.receive_group_sync(self.groupSyncReadCurrent)
        cur = self.read_from_group_sync(selected_IDs, self.groupSyncReadCurrent, ADDR_PRESENT_CURRENT, LEN_CURRENT)
        self.groupSyncReadCurrent.clearParam()
        return cur

    def get_errors(self, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        readings = []
        for selected_ID in selected_IDs:
            error_status, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, selected_ID, ADDR_ERROR)
            if result == COMM_SUCCESS:
                if error_status & 0b00000001:
                    readings.append([selected_ID, "Input voltage error"])
                if error_status & 0b00000100:
                    readings.append([selected_ID, "Overheating"])
                if error_status & 0b00001000:
                    readings.append([selected_ID, "Motor Encoder Error"])
                if error_status & 0b00010000:
                    readings.append([selected_ID, "Electrical shock"])
                if error_status & 0b00100000:
                    readings.append([selected_ID, "Overload"])
            else:
                print(f"Communication error: {error}")
        return readings

    def write_to_2bytes_group_sync(self, vals: list, ids: list, group_sync: GroupSyncWrite) -> None:
        # Queue up the messages to send
        for i, val in enumerate(vals):
            bytes = [DXL_LOBYTE(val), DXL_HIBYTE(val)]
            group_sync.addParam(ids[i], bytes)

    def write_to_4bytes_group_sync(self, vals: list, ids: list, group_sync: GroupSyncWrite) -> None:
        # Queue up the messages to send
        for i, val in enumerate(vals):
            bytes = [DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)), DXL_LOBYTE(DXL_HIWORD(val)), DXL_HIBYTE(DXL_HIWORD(val))]
            group_sync.addParam(ids[i], bytes)

    def send_group_sync(self, group_sync: GroupSyncRead):
        # Send
        dxl_comm_result = group_sync.txPacket()
        # Check comms
        if dxl_comm_result != COMM_SUCCESS:
            self._print_error_msg("Write position", dxl_comm_result=dxl_comm_result, dxl_error="", selected_ID="", print_only_if_error=True)
        group_sync.clearParam()

    def write_position(self, pos: list, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        self.write_to_4bytes_group_sync(pos, selected_IDs, self.groupSyncWritePosition)
        self.send_group_sync(self.groupSyncWritePosition)

    def write_velocity(self, vels: list, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        self.write_to_4bytes_group_sync(vels, selected_IDs, self.groupSyncWriteVelocity)
        self.send_group_sync(self.groupSyncWriteVelocity)

    def write_current(self, currents: list, ID=None):
        selected_IDs = self.fetch_and_check_ID(ID)
        self.write_to_2bytes_group_sync(currents, selected_IDs, self.groupSyncWriteCurrent)
        self.send_group_sync(self.groupSyncWriteCurrent)
