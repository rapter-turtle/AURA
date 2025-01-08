import struct


REQUEST_WRITE = 0x58
REQUEST_READ = 0x54

# write_command = struct.pack('40B',
# 	0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54,
# 	0x00, 0x00, # Reserved[2]
# 	0x00, 0x00, # PLC Info[2]
# 	0xA0, # CPU Info[1]
# 	0x33, # Source of Frame[1]
# 	0x01, 0x00, # Invoke ID[2]
# 	0x32, 0x00, # Length[2]  30+20=50, 50을 16진수로 표현하면 32이다.
# 	0x00, # Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module
# 	getCheckSum(write_header_buffer, 0, len(write_header_buffer)), #0xFF, checksum ToDo
# 	REQUEST_WRITE, 0x00, # Request[2]
# 	0x14, 0x00, # Data Type[2]
# 	0x00, 0x00, # Reserved[2]
# 	0x01, 0x00, # block length[2]
# 	0x08, 0x00, # block name length[2]
# 	0x25, 0x44, 0x42, 0x30, 0x30, 0x36, 0x30, 0x32, # D00301 -> %DB00602
# 	0x1E, 0x00 # PX4 to PLC Data Size
# )


# read_command = struct.pack('40B',
# 	0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54,
# 	0x00, 0x00, # Reserved[2]
# 	0x00, 0x00, # PLC Info[2]
# 	0xA0, # CPU Info[1]
# 	0x33, # Source of Frame[1]
# 	0x00, 0x00, # Invoke ID[2]
# 	0x14, 0x00, # Length[2]
# 	0x00, # Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module
# 	getCheckSum(read_header_buffer, 0, len(read_header_buffer)), # 0xFF, checksum ToDo
# 	REQUEST_READ, 0x00, # Request[2]
# 	0x14, 0x00, # Data Type[2]
# 	0x00, 0x00, # Reserved[2]
# 	0x01, 0x00, # block length[2]
# 	0x08, 0x00, # block name length[2]
# 	0x25, 0x44, 0x42, 0x30, 0x30, 0x38, 0x30, 0x32, #  D00401을 읽으려면 %DB00802 를 사용해야 한다.
# 	0x1E, 0x00 # PLC -> PX4 data size : 30을 16진수로 표현하면 1E이다.
# )

def getCheckSum(data, start, length):
	sum = 0
	for i in range(length):
		sum += data[start + i]
	sum = sum & 0x00FF
	return sum


class PlcToPx4Packet:
	def __init__(self):
		self.auto_control_status = 0
		self.emergency_stop_status = 0
		self.engine_rpm_status = 0
		self.clutch_status = 0
		self.steering_angle_status = 0
		self.trim_angle_status = 0
		self.empty1 = 0
		self.engine_running_status = 0
		self.bow_thruster_power_status = 0
		self.bow_thruster_rev_status = 0
		self.reserved1 = 0
		self.reserved2 = 0
		self.reserved3 = 0
		self.reserved4 = 0
		self.reserved5 = 0
	def parseDataBytes(self, data):
		arr = struct.unpack('15h', data)
		self.parseData(arr)

	def parseData(self, data):
		self.auto_control_status = data[0]
		self.emergency_stop_status = data[1]
		self.engine_rpm_status = data[2]
		self.clutch_status = data[3]
		self.steering_angle_status = data[4]
		self.trim_angle_status = data[5]
		self.empty1 = data[6]
		self.engine_running_status = data[7]
		self.bow_thruster_power_status = data[8]
		self.bow_thruster_rev_status = data[9]
		self.reserved1 = data[10]
		self.reserved2 = data[11]
		self.reserved3 = data[12]
		self.reserved4 = data[13]
		self.reserved5 = data[14]
	def printData(self):
		print('auto control status: ', self.auto_control_status)
		print('emergency stop status: ', self.emergency_stop_status)
		print('engine rpm status: ', self.engine_rpm_status)
		print('clutch status: ', self.clutch_status)
		print('steering angle status: ', self.steering_angle_status)
		print('trim angle status: ', self.trim_angle_status)
		print('empty1: ', self.empty1)
		print('engine running status: ', self.engine_running_status)
		print('bow thruster power status: ', self.bow_thruster_power_status)
		print('bow thruster rev status: ', self.bow_thruster_rev_status)
		print('reserved1: ', self.reserved1)
		print('reserved2: ', self.reserved2)
		print('reserved3: ', self.reserved3)
		print('reserved4: ', self.reserved4)
		print('reserved5: ', self.reserved5)

class Px4ToPlcPacket:
	def __init__(self):
		self.emtpy1 = 0
		self.empty2 = 0
		self.engine_thrust = 0
		self.clutch = 0
		self.steering_angle = 0
		self.trim_angle = 0
		self.empty3 = 0 
		self.engine_ignition = 0
		self.bow_thruster_power = 0
		self.bow_thruster_rev = 0
		self.reserved1 = 0
		self.reserved2 = 0
		self.reserved3 = 0
		self.reserved4 = 0
		self.reserved5 = 0

	def makePacket(self):
		return struct.pack('15h', self.emtpy1, self.empty2, self.engine_thrust, self.clutch, self.steering_angle, self.trim_angle, self.empty3, self.engine_ignition, self.bow_thruster_power, self.bow_thruster_rev, self.reserved1, self.reserved2, self.reserved3, self.reserved4, self.reserved5)   
					 
	def parseData(self, data):
		self.emtpy1 = data[0]
		self.empty2 = data[1]
		self.engine_thrust = data[2]
		self.clutch = data[3]
		self.steering_angle = data[4]
		self.trim_angle = data[5]
		self.empty3 = data[6]
		self.engine_ignition = data[7]
		self.bow_thruster_power = data[8]
		self.bow_thruster_rev = data[9]
		self.reserved1 = data[10]
		self.reserved2 = data[11]
		self.reserved3 = data[12]
		self.reserved4 = data[13]
		self.reserved5 = data[14]

	def printData(self):
		print('empty1: ', self.emtpy1)
		print('empty2: ', self.empty2)
		print('engine thrust: ', self.engine_thrust)
		print('clutch: ', self.clutch)
		print('steering angle: ', self.steering_angle)
		print('trim angle: ', self.trim_angle)
		print('empty3: ', self.empty3)
		print('engine ignition: ', self.engine_ignition)
		print('bow thruster power: ', self.bow_thruster_power)
		print('bow thruster rev: ', self.bow_thruster_rev)
		print('reserved1: ', self.reserved1)
		print('reserved2: ', self.reserved2)
		print('reserved3: ', self.reserved3)
		print('reserved4: ', self.reserved4)
		print('reserved5: ', self.reserved5)

class PLCPacket:
	def __init__(self):
		self.REQUEST_WRITE = 0x58
		self.REQUEST_READ = 0x54
		self.RESPONSE_WRITE = 0x59
		self.RESPONSE_READ = 0x55
		# self.write_header_buffer = [0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x33, 0x01, 0x00, 0x32, 0x00, 0x00,
		# 0xFF, 0x58, 0x00, 
		# 0x14, 0x00, 
		# 0x00, 0x00,
		# 0x01, 0x00, 0x08, 0x00, 0x25, 0x44, 0x42, 0x30, 0x30, 0x36, 0x30, 0x32, 0x1E, 0x00]
		self.write_header_buffer = [0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54,
		0x00, 0x00, # Reserved[2]
		0x00, 0x00, # PLC Info[2]
		0xA0, # CPU Info[1]
		0x33, # Source of Frame[1]
		0x01, 0x00, # Invoke ID[2]
		0x32, 0x00, # Length[2]  30+20=50, 50을 16진수로 표현하면 32이다.
		0x00, # Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module
		0xFF, #getCheckSum(write_header_buffer, 0, len(write_header_buffer)), #0xFF, checksum ToDo
		REQUEST_WRITE, 0x00, # Request[2]
		0x14, 0x00, # Data Type[2]
		0x00, 0x00, # Reserved[2]
		0x01, 0x00, # block length[2]
		0x08, 0x00, # block name length[2]
		0x25, 0x44, 0x42, 0x30, 0x30, 0x36, 0x30, 0x30, # D00301 -> %DB00602
		0x1E, 0x00 # PX4 to PLC Data Size
		]
		self.write_header_buffer[19] = getCheckSum(self.write_header_buffer, 0, 19)
        # self.read_header_buffer = [0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x33, 0x00, 0x00, 0x14, 0x00, 0x00]

		self.read_header_buffer = [0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54,
		0x00, 0x00, # Reserved[2]
		0x00, 0x00, # PLC Info[2]
		0x00, # CPU Info[1]
		0x33, # Source of Frame[1]
		0x00, 0x00, # Invoke ID[2]
		0x12, 0x00, # Length[2]
		0x00, # Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module
		0x00, #getCheckSum(self.read_header_buffer, 0, len(self.read_header_buffer)), # 0xFF, checksum ToDo
		REQUEST_READ, 0x00, # Request[2]
		0x14, 0x00, # Data Type[2]
		0x00, 0x00, # Reserved[2]
		0x01, 0x00, # block length[2]
		0x06, 0x00, # block name length[2] 34 30 30 0600  
		0x25, 0x44, 0x42, 0x38, 0x30,  0x30, #  D00401을 읽으려면 %DB00802 를 사용해야 한다.
		0x1E, 0x00 # PLC -> PX4 data size : 30을 16진수로 표현하면 1E이다.
		]

		# self.read_header_buffer[19] = getCheckSum(self.read_header_buffer, 0, 19)

		self.read_response_header_buffer = [0x4C, 0x53, 0x49, 0x53, 0x2D, 0x58, 0x47, 0x54,
		0x00, 0x00, # Reserved[2]
		0x00, 0x00, # PLC Info[2]
		0xA0, # CPU Info[1]
		0x33, # Source of Frame[1]
		0x00, 0x00, # Invoke ID[2]
		0x14, 0x00, # Length[2]
		0x00, # Bit0~3 : slot # of FEnet I/F module, Bit4~7 : base # of FEnet I/F module
		0xFF, #getCheckSum(), # 0xFF, checksum ToDo
		self.RESPONSE_READ, 0x00, # Request[2]
		0x14, 0x00, # Data Type[2]
		0x00, 0x00, # Reserved[2]
		0x01, 0x00, # block length[2]
		0x08, 0x00, # block name length[2]
		0x1E, 0x00 # PLC -> PX4 data size : 30을 16진수로 표현하면 1E이다.
		]
		self.read_response_header_buffer[19] = self.getCheckSum(self.read_response_header_buffer, 0, 19)

	def unpackWritePacket(self, buffer):
		header, data = struct.unpack('40B30B', buffer)

	def unpackReadPacket(self, buffer):
		header = struct.unpack('40B', buffer)
	
	def unpackReadRespondPacket(self, buffer):
		header, data = struct.unpack('40B30B', buffer)
	
	def getCheckSum(self, data, start, length):
		sum = 0
		for i in range(start, start + length):
			sum += data[i]
		sum = sum & 0x00FF
		return sum

	def makeWritePacket2(self, throttle, steering, clutch):
		px4ToplcPacket = Px4ToPlcPacket()
		px4ToplcPacket.engine_thrust = throttle
		px4ToplcPacket.clutch = clutch
		px4ToplcPacket.steering_angle = steering
		px4ToplcPacket.trim_angle = 4
		px4ToplcPacket.engine_ignition = 5
		px4ToplcPacket.bow_thruster_power = 6
		px4ToplcPacket.bow_thruster_rev = 7
		return struct.pack('40B15H', *self.write_header_buffer, 
						   px4ToplcPacket.emtpy1, px4ToplcPacket.empty2, px4ToplcPacket.engine_thrust, px4ToplcPacket.clutch, px4ToplcPacket.steering_angle, px4ToplcPacket.trim_angle, px4ToplcPacket.empty3, px4ToplcPacket.engine_ignition, px4ToplcPacket.bow_thruster_power, px4ToplcPacket.bow_thruster_rev, px4ToplcPacket.reserved1, px4ToplcPacket.reserved2, px4ToplcPacket.reserved3, px4ToplcPacket.reserved4, px4ToplcPacket.reserved5)


	def makeWritePacket(self):
		px4ToplcPacket = Px4ToPlcPacket()
		px4ToplcPacket.engine_thrust = 1
		px4ToplcPacket.clutch = 2
		px4ToplcPacket.steering_angle = 3
		px4ToplcPacket.trim_angle = 4
		px4ToplcPacket.engine_ignition = 5
		px4ToplcPacket.bow_thruster_power = 6
		px4ToplcPacket.bow_thruster_rev = 7
		return struct.pack('40B15H', *self.write_header_buffer, 
						   px4ToplcPacket.emtpy1, px4ToplcPacket.empty2, px4ToplcPacket.engine_thrust, px4ToplcPacket.clutch, px4ToplcPacket.steering_angle, px4ToplcPacket.trim_angle, px4ToplcPacket.empty3, px4ToplcPacket.engine_ignition, px4ToplcPacket.bow_thruster_power, px4ToplcPacket.bow_thruster_rev, px4ToplcPacket.reserved1, px4ToplcPacket.reserved2, px4ToplcPacket.reserved3, px4ToplcPacket.reserved4, px4ToplcPacket.reserved5)
	
	def makeReadPacket(self):
		# return struct.pack('40B', *self.read_header_buffer)
		return struct.pack(str(len(self.read_header_buffer))+'B', *self.read_header_buffer)

	def makeReadRespondPacket(self):
		plcTopx4Packet = PlcToPx4Packet()
		plcTopx4Packet.auto_control_status = 1
		plcTopx4Packet.engine_rpm_status = 2
		plcTopx4Packet.clutch_status = 3
		plcTopx4Packet.steering_angle_status = 4
		plcTopx4Packet.trim_angle_command = 5
		plcTopx4Packet.engine_running_status = 6
		plcTopx4Packet.bow_thruster_power_status = 7
		plcTopx4Packet.bow_thruster_rev_status = 8
		return struct.pack('32B15h', *self.read_response_header_buffer, 
						   plcTopx4Packet.auto_control_status, plcTopx4Packet.empty1, plcTopx4Packet.engine_rpm_status, plcTopx4Packet.clutch_status, plcTopx4Packet.steering_angle_status, plcTopx4Packet.trim_angle_command, plcTopx4Packet.empty2, plcTopx4Packet.engine_running_status, plcTopx4Packet.bow_thruster_power_status, plcTopx4Packet.bow_thruster_rev_status, plcTopx4Packet.reserved1, plcTopx4Packet.reserved2, plcTopx4Packet.reserved3, plcTopx4Packet.reserved4, plcTopx4Packet.reserved5)

