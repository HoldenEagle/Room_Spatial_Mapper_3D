import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import open3d as o3d
from PyQt6.QtWidgets import QApplication, QDialog, QLabel, QVBoxLayout, QPushButton , QMessageBox, QWidget, QHBoxLayout
from PyQt6.QtGui import QColor
import sys
import math
import time





class PointCloudWindow():
    def __init__(self , pcd_file):
        pcd = o3d.io.read_point_cloud(pcd_file)

        self.pcd = pcd.voxel_down_sample(voxel_size = .034)


        self.points = np.array(self.pcd.points)
        self.colors = np.array(self.pcd.colors)

        # Movement speed for translation
        self.move_speed = 0.8

        # Mouse rotation parameters
        self.rotate_speed = 3.0
        self.rotate_sensitivity = 0.2
        self.last_mouse_pos = None

        # Font initialization for labels
        pygame.font.init()
        self.font = pygame.font.SysFont('Ariel', 20)  # You can customize the font and size here

        self.location_is_on = False
        self.location_window = None
        
        self.tool_window = None
        
        self.compute_distances_on = False
        self.center = np.mean(self.points , axis=0)
        self.camera_pos = np.array([0.0, 0.0, 5.0]) + self.center
        
        camera_front = self.center - self.camera_pos
        self.camera_front = camera_front / np.linalg.norm(camera_front)
        self.camera_up = [0, 1, 0]
        
        self.camera_pos_orig = self.camera_pos
        self.camera_front_orig = self.camera_front
        self.camera_up_orig = self.camera_up
        
        self.x_value = self.camera_pos[0]
        self.y_value = self.camera_pos[1]
        self.z_value = self.camera_pos[2]
        
        self.x_orig = self.camera_pos[0]
        self.y_orig = self.camera_pos[1]
        self.z_orig = self.camera_pos[2]
    
        self.distance_points = []
        self.original_colors = []
        self.indices = []
    
    def setView(self):
        glLoadIdentity()
        gluLookAt(self.camera_pos[0], self.camera_pos[1], self.camera_pos[2], 
                  self.camera_front[0], self.camera_front[1], self.camera_front[2], 
                  self.camera_up[0], self.camera_up[1], self.camera_up[2])

    
    def computePointDistanceFlag(self , parent_window):
        if parent_window:
            parent_window.close()
        
        #Give them a message box to let them know what's going on
        QMessageBox.information(None , 'Point Cloud Distances', 'Click on two points to get the distance between them. Once you choose your two points, press enter to see a window showing the distance.')
        self.compute_distances_on = True
        return
    
    def open_toolbar(self):
        self.tool_window = QWidget()
        self.tool_window.setWindowTitle("Toolbar")
        self.tool_window.setGeometry(400, 400, 400, 400)  # (x, y, width, height)

        color = QColor(255, 255, 255)
        self.tool_window.setStyleSheet(f'background-color: {color.name()}')

        distance_button = QPushButton("Compute distances")
        distance_button.setStyleSheet('QPushButton { font-size: 12px; padding: 5px; font-weight: bold;color: white; background-color: blue;}')
        distance_button.clicked.connect(lambda: self.computePointDistanceFlag(self.tool_window))
        distance_button.setFixedSize(185 , 40)
        
        
        tool_layout = QVBoxLayout(self.tool_window)
        tool_layout.addWidget(distance_button)
        
        
        self.tool_window.show()
    
    
    
    def change_location(self):
        self.location_is_on = False
        return
    
    def open_location(self, x_value , y_value , z_value , shouldOPEN):
        if not shouldOPEN:
            if self.location_window:
                self.location_window.close()
            return
        self.location_window = QWidget()
        self.location_window.setWindowTitle("Location")
        self.location_window.setGeometry(60 , 70 , 60 , 70)
    
        color = QColor(255 , 255, 255)
        self.location_window.setStyleSheet(f'background-color: {color.name()}')

        # Create layout
        main_layout = QVBoxLayout()
    
        # X coordinate
        x_layout = QHBoxLayout()
        x_label = QLabel("X:")
        x_value_label = QLabel(str(x_value))  # Replace with actual X coordinate value
        x_layout.addWidget(x_label)
        x_layout.addWidget(x_value_label)
        main_layout.addLayout(x_layout)
    
        # Y coordinate
        y_layout = QHBoxLayout()
        y_label = QLabel("Y:")
        y_value_label = QLabel(str(y_value))  # Replace with actual Y coordinate value
        y_layout.addWidget(y_label)
        y_layout.addWidget(y_value_label)
        main_layout.addLayout(y_layout)
    
        # Z coordinate
        z_layout = QHBoxLayout()
        z_label = QLabel("Z:")
        z_value_label = QLabel(str(z_value))  # Replace with actual Z coordinate value
        z_layout.addWidget(z_label)
        z_layout.addWidget(z_value_label)
        main_layout.addLayout(z_layout)
    
        # Set main layout for the window
        self.location_window.setLayout(main_layout)
        
    
        #handle it being closed early
        #location_window.destroyed.connect(self.change_location)
    
        self.location_window.show()


    # Function to reset the view
    def reset_view(self):
        glLoadIdentity()
        glTranslatef(-np.mean(self.points[:, 0]), -np.mean(self.points[:, 1]), -np.mean(self.points[:, 2]) - 30.0)  # Adjust translation for centering and distance
        glScalef(6.0, 6.0, 6.0)  # Scale up by a factor of 6


    #draw points on the cloud
    def draw_points(self):
        glPointSize(5)
        glBegin(GL_POINTS)
        for i in range(len(self.points)):
            glColor3fv(self.colors[i])  # Set color for the current point
            glVertex3fv(self.points[i])  # Draw the point at its coordinates
        glEnd()
        
    texture_id = None
    def declare_text(self):
        global texture_id
        # Label positions
        label_x = (1.0, 0.0, 0.0)
        label_y = (0.0, 1.0, 0.0)
        label_z = (0.0, 0.0, 1.0)
        
        texture_id = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, texture_id)
        
        # Render text labels for axes
        text_x = self.font.render('X', True, (255, 0, 0))
        text_y = self.font.render('Y', True, (0, 255, 0))
        text_z = self.font.render('Z', True, (0, 0, 255))

        width, height = text_x.get_width(), text_x.get_height()
        texture_data = pygame.image.tostring(text_x, 'RGBA', True)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_data)
        
        
    
    def draw_axes(self):
        #glBindTexture(GL_TEXTURE_2D, texture_id)
        glBegin(GL_LINES)
        glColor3f(1.0, 0.0, 0.0)  # Red X-axis
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(10.0, 0.0, 0.0)
        glColor3f(0.0, 1.0, 0.0)  # Green Y-axis
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 10.0, 0.0)
        glColor3f(0.0, 0.0, 1.0)  # Blue Z-axis
        glVertex3f(0.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 10.0)
        glEnd()
        
        
    
    # Function to draw the toolbar using OpenGL
    def draw_toolbar(Window_Size, toolbar_height):
        glBegin(GL_QUADS)
        glColor3f(0.8, 0.8, 0.8)  # Light gray background for toolbar
        glVertex2f(0, 0)
        glVertex2f(Window_Size[0], 0)
        glVertex2f(Window_Size[0], toolbar_height)
        glVertex2f(0, toolbar_height)
        glEnd()
    
    
    def rotate_left(self , angle):
        angle_rad = math.radians(angle)
        
        new_front_x = self.camera_front[0] * math.cos(angle_rad) - self.camera_front[2] * math.sin(angle_rad)
        
        new_front_z = self.camera_front[0] * math.sin(angle_rad) + self.camera_front[2] * math.cos(angle_rad)
    
        self.camera_front = [new_front_x, self.camera_front[1], new_front_z]
        
    def rotate_right(self, angle):
        # Convert angle to radians
        angle_rad = math.radians(angle)
    
        # Rotate around the up vector
        new_front_x = self.camera_front[0] * math.cos(-angle_rad) - self.camera_front[2] * math.sin(-angle_rad)
        new_front_z = self.camera_front[0] * math.sin(-angle_rad) + self.camera_front[2] * math.cos(-angle_rad)
    
        self.camera_front = [new_front_x, self.camera_front[1], new_front_z] 
    
    def rotate_up(self , angle):
        # Convert angle to radians
        angle_rad = math.radians(angle)
    
        # Rotate around the right vector
        new_front_y = self.camera_front[1] * math.cos(angle_rad) + self.camera_up[1] * math.sin(angle_rad)
        new_front_z = self.camera_front[2] * math.cos(angle_rad) + self.camera_up[2] * math.sin(angle_rad)
    
        self.camera_front = [self.camera_front[0], new_front_y, new_front_z]
    
    def rotate_down(self,angle):
        # Convert angle to radians
        angle_rad = math.radians(angle)
    
        # Compute right vector (cross product of front and up)
        right = np.cross(self.camera_front, self.camera_up)
    
        # Rotate around the right vector
        new_front_y = self.camera_front[1] * math.cos(-angle_rad) + self.camera_up[1] * math.sin(-angle_rad)
        new_front_z = self.camera_front[2] * math.cos(-angle_rad) + self.camera_up[2] * math.sin(-angle_rad)
    
        self.camera_front = [self.camera_front[0], new_front_y, new_front_z]
    
    def convert_screen_to_world(self, x, y):
        modelview = glGetDoublev(GL_MODELVIEW_MATRIX)
        projection = glGetDoublev(GL_PROJECTION_MATRIX)
        viewport = glGetIntegerv(GL_VIEWPORT)
        winX = float(x)
        winY = float(viewport[3] - y)
        winZ = glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT).item()
        world_coords = gluUnProject(winX, winY, winZ, modelview, projection, viewport)
        return np.array(world_coords[:3])
    
    def find_nearest_point(self, world_coords):
        distances = np.linalg.norm(self.points - world_coords, axis=1)
        nearest_index = np.argmin(distances)
        return nearest_index

    # Handle user input for movement
    def handle_input(self):
        #creates an instance of an application, allowing us to have an event loop and stuff
        mouse = pygame.mouse.get_pressed() 
        if mouse[0]:
            
            mouseX, mouseY = pygame.mouse.get_pos()
            world_cords = self.convert_screen_to_world(mouseX, mouseY)
            nearest_point = self.find_nearest_point(world_cords)
             
            if self.compute_distances_on:
                if len(self.distance_points) != 2:
                    self.distance_points.append(np.copy(self.points[nearest_point]))
                    self.indices.append(nearest_point)
                    self.original_colors.append(np.copy(self.colors[nearest_point]))
                    self.colors[nearest_point] = [255 , 0 ,0]
                elif len(self.distance_points) >= 2:
                    QMessageBox.information(None , 'Error' , "You have already chosen two points. Press enter to get distance")
                        
        keys = pygame.key.get_pressed()
        if keys[K_LEFT]:
            glTranslatef(-self.move_speed, 0.0, 0.0)
            self.x_value -= self.move_speed
            self.camera_pos[0] += self.move_speed
            self.open_location(self.x_value , self.y_value , self.z_value , self.location_is_on)
        elif keys[K_RIGHT]:
            glTranslatef(self.move_speed, 0.0, 0.0)
            self.x_value += self.move_speed
            self.camera_pos[0] -= self.move_speed
            self.open_location(self.x_value , self.y_value , self.z_value ,self.location_is_on)
        elif keys[K_UP]:
            glTranslatef(0.0, self.move_speed, 0.0)
            self.y_value += self.move_speed
            self.camera_pos[1] -= self.move_speed
            self.open_location(self.x_value , self.y_value , self.z_value ,self.location_is_on)
        elif keys[K_DOWN]:
            glTranslatef(0.0, -self.move_speed, 0.0)
            self.y_value -= self.move_speed
            self.camera_pos[1] += self.move_speed
            self.open_location(self.x_value , self.y_value , self.z_value , self.location_is_on)
        elif keys[K_EQUALS]:
            glTranslatef(0.0 , 0.0 , self.move_speed)
            self.z_value += self.move_speed
            self.camera_pos[2] -= self.move_speed
            self.open_location(self.x_value , self.y_value , self.z_value , self.location_is_on)
        elif keys[K_MINUS]:
            glTranslatef(0.0 , 0.0 , -self.move_speed)
            self.z_value -= self.move_speed
            self.camera_pos[2] += self.move_speed
            self.open_location(self.x_value , self.y_value , self.z_value , self.location_is_on)
        elif keys[K_1]:
            glRotatef(-1, 0, 1, 0)
        elif keys[K_2]:
            glRotatef(1, 0, 1, 0)
        elif keys[K_3]:
            glRotatef(1, 1, 0, 0)
        elif keys[K_4]:
            glRotatef(-1, 1, 0, 0)
        elif keys[K_a]:
            self.rotate_left(10)
            self.setView()
        elif keys[K_d]:
            self.rotate_right(10)
            self.setView()
        elif keys[K_w]:
            self.rotate_up(10)
            self.setView()
        elif keys[K_s]:
            self.rotate_down(10)
            self.setView()
        elif keys[K_SPACE]:
            self.reset_view()
            self.open_location(self.x_orig , self.y_orig , self.z_orig , self.location_is_on)
        elif keys[K_q]:
            pygame.quit()
            sys.exit()
        elif keys[K_t]:
            #open toolbar window
            self.open_toolbar()
        elif keys[K_l]:
            self.location_is_on = True if self.location_is_on is False else False
            self.open_location(self.x_value , self.y_value , self.z_value , self.location_is_on)
        elif keys[K_5]:
            if self.compute_distances_on:
                point1 , point2 = np.array(self.distance_points[0]) , np.array(self.distance_points[1])
                distance = np.sqrt(np.sum((point1 - point2)**2))
                QMessageBox.information(None , 'Distance of Points' , f"Distance {distance}")
                
                for coordinate_index in range(len(self.original_colors)):
                    self.colors[self.indices[coordinate_index]] = self.original_colors[coordinate_index]
                    
                self.indices = []
                self.distance_points = []
                self.original_colors = []
                

    
    def run(self):
        #create Window size
        Window_Size = (750 , 750)
        toolbar_height = 10
    
        pygame.init() #initializes pygame attributes
        pygame.display.set_caption("Room Viewer")
        

        screen = pygame.display.set_mode(Window_Size, DOUBLEBUF | OPENGL)
        clock = pygame.time.Clock()

        instruction_bar_height = 50
    
        # Set clear color to grey (R, G, B values normalized to [0, 1])
        glClearColor(0.5, 0.5, 0.5, 1.0)  # Grey color
    
        #gives us perspective on how to view it
        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, (Window_Size[0] / (Window_Size[1] - instruction_bar_height)), 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)
    
    
        # Translate and scale the point cloud to center and enlarge it
        glLoadIdentity()
        glTranslatef(-np.mean(self.points[:, 0]), -np.mean(self.points[:, 1]), -np.mean(self.points[:, 2]) - 30.0)  # Adjust translation for centering and distance
        glScalef(6.0, 6.0, 6.0)  # Scale up by a factor of 10

        self.setView()
    
        #open_location(self.x_value, self.y_value , self.z_value)
        #Begin event running for the game
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
                
            glClear(GL_COLOR_BUFFER_BIT) #clear out the buffers
        
            self.handle_input()
            #self.declare_text()
            self.draw_axes()
            self.draw_points()
            #draw_toolbar(Window_Size, toolbar_height)
            
            pygame.display.flip()
            clock.tick(60.0)

if __name__ == "__main__":
    # Start the event loop
    app = QApplication(sys.argv)

    newScreen = PointCloudWindow("cloud_bin_0.ply")
    newScreen.run()
    sys.exit(app.exec())

    
    
    

