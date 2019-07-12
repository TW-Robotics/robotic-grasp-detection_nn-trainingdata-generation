import sys
import rospy

''' Class handles goals of MiR and UR '''
''' To teach a new goal, send the MiR to a goal in the web-interface and
    write x, y coordinates and orientation down. Then measure the height
    of the table or platform and enter the orientation where the UR should
    search. Give the goal a name and add it to createGoal(). You can then
    call it with this name from robot_control.py '''

class goal():
	# Init empty goal
	def __init__(self, goalName):
		self.name = "invalid"
		self.posx = 0 		# meters
		self.posy = 0 		# meters
		self.rz = 0 		# degrees
		self.height = 0 	# mm
		self.orientation = "invalid"	# left, right or front
		
		# Fill goal with useful information
		self.createGoal(goalName)

	# Check if goal-name exists and make goal
	def createGoal(self, goalName):
		if goalName == "stationrr":
			self.name = "Robot station high right"
			self.posx = 13.9
			self.posy = 3.55
			self.rz = 12
			self.height = 720
			self.orientation = "right"
		elif goalName == "stationlf":
			self.name = "Robot station high left"
			self.posx = 16
			self.posy = 4.85
			self.rz = -48
			self.height = 720
			self.orientation = "front"
		elif goalName == "stationll":
			self.name = "Robot station high left"
			self.posx = 16.1
			self.posy = 4.6
			self.rz = -137
			self.height = 720
			self.orientation = "front"
		elif goalName == "stationnf":
			self.name = "Robot station low"
			self.posx = 15.75
			self.posy = 10.35
			self.rz = 100
			self.height = 990
			self.orientation = "front"
		return self

	# Print information of goal
	def display(self):
		print " Goal: ", self.name, "Height: ", str(self.height), "mm"
		print "  MIR: ", str(self.posx), str(self.posy), str(self.rz)
		print "   UR: ", self.orientation

def main(args):
	newm = goal("table1")
	newm.display()

if __name__ == '__main__':
	main(sys.argv)
