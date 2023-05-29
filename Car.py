import tkinter as tk
from matplotlib.patches import Circle, Rectangle
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import math
import os
from matplotlib import animation
from rbf import *
from PSO import *

class Car():
	def __init__(self):
		self.x = None
		self.y = None
		self.Phi = 90	# car angel: calcalated from mathematic
		self.Theta = 0 	# wheel angel: received from prediction
		self.radius = 6
		self.wheel_max = 40
		self.wheel_min = -40
		self.car_max = 270
		self.car_min = -90

		with open("coordinate.txt", "r") as f:
			self.lines = f.readlines()

	def init_path(self):
		self.wall_x = []
		self.wall_y = []
		self.end_x = []
		self.end_y = []
		for num, line in enumerate(self.lines):
			xdata = line.split(",")
			if (num > 2):
				self.wall_x.append(int(xdata[0]))
				self.wall_y.append(int(xdata[1]))
			elif (num > 0):
				self.end_x.append(int(xdata[0]))
				self.end_y.append(int(xdata[1]))
			else:
				self.x = int(xdata[0])
				self.y = int(xdata[1])
				self.Phi = float(xdata[2])
				self.wheel = Circle((self.x, self.y), 3, fill=False)
	
	def CreateWidgets(self, root):
		""" Display distances """
		tk.Label(text="Left: ", font=('Comic Sans MS', 12)).grid(row=6, column=0)
		self.dl = tk.DoubleVar() 
		l_d = tk.Label(root, textvariable=self.dl, font=('Comic Sans MS', 12))
		l_d.grid(row=6, column=1)

		tk.Label(text="Front: ", font=('Comic Sans MS', 12)).grid(row=6, column=2)
		self.df = tk.DoubleVar() 
		f_d = tk.Label(root, textvariable=self.df, font=('Comic Sans MS', 12))
		f_d.grid(row=6, column=3)

		tk.Label(text="Right: ", font=('Comic Sans MS', 12)).grid(row=7, column=0)
		self.dr = tk.DoubleVar() 
		r_d = tk.Label(root, textvariable=self.dr, font=('Comic Sans MS', 12))
		r_d.grid(row=7, column=1)

	def draw_path(self, ax):
		""" Path and start point """
		ax.plot(self.wall_x, self.wall_y)
		rec = Rectangle((self.end_x[0], self.end_y[0]), abs(self.end_x[1]-self.end_x[0]), -abs(self.end_y[1]-self.end_y[0]))
		ax.add_patch(rec)

	def reset(self):
		self.x = None
		self.y = None
		self.Phi = 90
		self.Theta = 0

	def move(self, network, ax, ani=None, visibility=False):
		""" whether reach end rec or not """
		a, b, c = self.getLine(self.end_x[0], self.end_y[0], self.end_x[1], self.end_y[1])
		if a * self.x + b * self.y + c > 0:
			print("completed.")
			if visibility:
				ani.pause()			
			if not visibility:
				return "completed"

		""" detect and record distances & coordinate """
		front = self.detect_distance(ax, self.x, self.y, 'front')
		self.df.set(round(front, 2))
		left = self.detect_distance(ax, self.x, self.y, 'left')
		self.dl.set(round(left, 2))
		right = self.detect_distance(ax, self.x, self.y, 'right')
		self.dr.set(round(right, 2))
		
		""" the next wheel degree """
		data = [front, right, left]
		degree = network.forward(data).item()
		self.Theta = degree
		if self.Theta > self.wheel_max:
			self.Theta = self.wheel_max
		if self.Theta < self.wheel_min:
			self.Theta = self.wheel_min
		
		# print("[wheel]: ", self.Theta, " degree.")
		# print("[car]: ", self.Phi, " degree.")		

		""" move car """
		self.Phi = math.radians(self.Phi)
		self.Theta = math.radians(self.Theta)
		self.x += math.cos(self.Phi + self.Theta) + math.sin(self.Theta) * math.sin(self.Phi)
		self.y += math.sin(self.Phi + self.Theta) - math.sin(self.Theta) * math.cos(self.Phi)
		
		wheel = Circle((self.x, self.y), self.radius/2, fill=False)
		ax.add_patch(wheel)
		ax.plot(self.x, self.y, color='red', marker='.')
		
		""" car degree """
		self.Phi -= math.asin(2 * math.sin(self.Theta) / self.radius)
		self.Phi = math.degrees(self.Phi)
		self.Theta = math.degrees(self.Theta)

		""" collision test """
		if self.collision():
			# print("collision.")
			if visibility:
				ani.pause()			
			if not visibility:
				return "collision"

		return "continue"
			
	def collision(self):
		for i in range(len(self.wall_x)-1):
			a, b, c = self.getLine(self.wall_x[i], self.wall_y[i], self.wall_x[i+1], self.wall_y[i+1])
			# L' = bx-ay+c1=0
			a1, b1 = b, -a
			c1 = -a1 * self.x - b1 * self.y
			x_, y_ = self.IntersectPoint(a, b, c, a1, b1, c1)
			if self.wall_constrain(self.wall_x[i], self.wall_y[i], self.wall_x[i+1], self.wall_y[i+1], x_, y_):
				if self.cal_distance(x_, y_, self.x, self.y) <= self.radius/2:
					return True
		return False

	def detect_distance(self, ax, x, y, point='center'):
		if point == 'right':
			right_angle = self.Phi - 45
			new_x, new_y = self.point_on_circle(x, y, self.radius/2, right_angle)
		elif point == 'left':
			left_angle = self.Phi + 45
			new_x, new_y = self.point_on_circle(x, y, self.radius/2, left_angle)
		elif point == 'front':
			new_x, new_y = self.point_on_circle(x, y, self.radius/2, self.Phi)
		else:
			new_x = x
			new_y = y

		a, b, c = self.getLine(new_x, new_y, x, y)
		inter_x, inter_y = self.WallsIntersection(a, b, c)
		# ax.plot([self.x, inter_x], [self.y, inter_y])

		return self.cal_distance(inter_x, inter_y, self.x, self.y)

	def point_on_circle(self, x, y, r, angle):
		rad = math.radians(angle)
		new_x = x + r * math.cos(rad)
		new_y = y + r * math.sin(rad)
		return new_x, new_y

	def getLine(self, x1, y1, x2, y2):
		# ax+by+c
		sign = 1
		a = y2 - y1
		if a < 0:
			sign = -1
			a = sign * a
		b = sign * (x1 - x2)
		c = sign * (y1 * x2 - x1 * y2)
		return a, b, c

	def WallsIntersection(self, a, b, c):
		# line: ax+by+c=0
		inter_x = self.x
		inter_y = self.y
		for i in range(len(self.wall_x)-1):
			a1, b1, c1 = self.getLine(self.wall_x[i], self.wall_y[i], self.wall_x[i+1], self.wall_y[i+1])
			x_, y_ = self.IntersectPoint(a, b, c, a1, b1, c1)
			if self.intersect_constraint(x_, y_) and self.wall_constrain(self.wall_x[i], self.wall_y[i], self.wall_x[i+1], self.wall_y[i+1], x_, y_):
				if self.cal_distance(x_, y_, self.x, self.y) < self.cal_distance(inter_x, inter_y, self.x, self.y):
					inter_x = x_
					inter_y = y_
				if inter_x == self.x and inter_y == self.y:
					inter_x = x_
					inter_y = y_

		return inter_x, inter_y

	def intersect_constraint(self, x, y):
		# constrain equation F
		left_x, left_y = self.point_on_circle(self.x, self.y, self.radius, self.Phi+90)
		right_x, right_y = self.point_on_circle(self.x, self.y, self.radius, self.Phi-90)
		a, b, c = self.getLine(left_x, left_y, right_x, right_y)
		if a * x + b * y + c > 0:
			return True
		else: 
			return False

	def wall_constrain(self, x1, y1, x2, y2, x_, y_):
		# constrain to a line not to an equation
		if x_ <= max(x1, x2) and x_ >= min(x1, x2) and y_ <= max(y1, y2) and y_ >= min(y1, y2):
			return True
		else:
			return False

	def cal_distance(self, x1, y1, x2, y2):
		return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

	def IntersectPoint(self, a, b, c, a1, b1, c1):
		# ax+by+c=0 -> ax+by=c
		A = np.array([[a, b], [a1, b1]])
		B = np.array([-c, -c1]).reshape(2, 1)
		ans = np.linalg.solve(A, B)
		return ans[0].item(), ans[1].item()

def animate(i):
	ax.clear()
	car.draw_path(ax)
	car.move(network, ax, ani=anim, visibility=True)

	move_path.append([car.x, car.y])
	ax.plot([i[0] for i in move_path], [i[1] for i in move_path], color='gray')
	canvas.draw()

if __name__ == "__main__":
	root = tk.Tk()
	root.title("Particle Swarm Optimization")

	figure = Figure(figsize=(6,6), dpi=100)
	ax = figure.add_subplot(111)
	ax.set_title("Particle Swarm Optimization")

	canvas = FigureCanvasTkAgg(figure, master=root)
	canvas.draw()
	canvas.get_tk_widget().grid(row=0, column=0, columnspan=4)
	
	car = Car()
	car.CreateWidgets(root)
	car.init_path()
	car.draw_path(ax)

	# init particle, particle=(k*dim + 2*k + 1, )
	particle_group = 300
	rbfn_k = 5
	input_dim = 3
	particle_size = rbfn_k * input_dim + 2* rbfn_k + 1
	# particles = np.random.randn(particle_group, particle_size)
	particles = np.random.uniform(-10, 10, (particle_group, particle_size))
	arrive_particle = None
	# print("before: ", particles)

	filename = "PSO.txt"
	if os.path.exists(filename):
		os.remove(filename)

	PSO_process = PSO(particles)

	iteration = 40
	scores = np.zeros((particle_group, iteration))
	for i in range(iteration):
		for p in range(particle_group):
			network = rbf(particles[p], input_dim, rbfn_k)
			while True:
				end = car.move(network, ax)
				if end == "completed":
					scores[p][i] += 100
					arrive_particle = np.copy(particles[p])
					with open(filename, "a") as f:
						f.write(str(particles[p]) + "\n")
					break
				elif end == "collision":
					break
				else:
					scores[p][i] += 1

			ax.clear()
			car.reset()
			car.init_path()
		
		PSO_process.evaluate_fitness(scores, i)
		phi_1, phi_2 = 1.4, 1.0
		PSO_process.velocity_update(phi_1, phi_2)
		PSO_process.position_update()	

		print("[{}], mean score: {:.2f}".format(i, sum(scores[:, i])/scores.shape[0]))
	
	particles = np.copy(PSO_process.x)

	if isinstance(arrive_particle, np.ndarray):
		network = rbf(arrive_particle, input_dim, rbfn_k)
	else:
		index = np.where(scores[:, i]==max(scores[:, i]))[0][0]
		network = rbf(particles[index], input_dim, rbfn_k)
		print(particles[index])

	car.reset()
	car.init_path()

	# final
	move_path = [[car.x, car.y]]
	anim = animation.FuncAnimation(figure, animate, frames=100, interval=100)
	root.mainloop()
