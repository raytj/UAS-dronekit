from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
from datetime import datetime
import digi.xbee.devices

class xbeeDevice(XBeeDevice):
	def __init__(self, *args, address=None, uasNode=None):
		super(xbeeDevice, self).__init__(*args)
		self.address = address
		self.uasNode = uasNode

	def xbeeSend(self, REMOTE_NODE_ID, DATA_TO_SEND):
		if self.is_open():
			# Obtain the remote XBee self.xbeeDevice from the XBee network.
			xbee_network = self.xbeeDevice.get_network()
			remote_device = xbee_network.discover_device(REMOTE_NODE_ID)
			if remote_device is None:
					print("Could not find the remote device")
					exit(1)

			print("Sending data to %s >> %s...\n" % (remote_device.get_64bit_addr(), DATA_TO_SEND))

			self.xbeeDevice.send_data(remote_device, DATA_TO_SEND)

			print("Success")
		else:
			print("error, xbee device not open\n")


	def xbee_on_message(self, name):
		if self.is_open():
			def data_receive_callback(xbee_message):
					self.message_available = "%s, %s".format(xbee_message.remote_device.get_64bit_addr(), xbee_message.data.decode())

			self.add_data_received_callback(data_receive_callback)
		else:
			print("error, xbee device not open\n")

class droneVehicle(Vehicle):
	def __init__(self, *args):
		super(droneVehicle, *args).__init__(*args)

	def xbeeInit(self, port, baud, address=None, uasNode=None):
		self.xbeeDevice = xbeeDevice(port, baud, address, uasNode)
		self.xbeeDevice.open()
