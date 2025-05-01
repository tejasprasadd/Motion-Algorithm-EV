import pygame

class Rectangle:
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    
    def contains(self, point):
        """Check if the rectangle contains a point"""
        return (point.x >= self.x - point.radius and
                point.x <= self.x + self.width + point.radius and
                point.y >= self.y - point.radius and
                point.y <= self.y + self.height + point.radius)
    
    def intersects(self, range_rect):
        """Check if this rectangle intersects with another rectangle"""
        return not (range_rect.x > self.x + self.width or
                    range_rect.x + range_rect.width < self.x or
                    range_rect.y > self.y + self.height or
                    range_rect.y + range_rect.height < self.y)

class QuadTree:
    def __init__(self, boundary, capacity=4, max_depth=6, depth=0):
        self.boundary = boundary
        self.capacity = capacity  # Maximum number of points before subdivision
        self.points = []  # Points in this quadtree node
        self.divided = False  # Whether this node has been subdivided
        self.max_depth = max_depth  # Maximum depth of the tree
        self.depth = depth  # Current depth of this node
        self.northwest = None
        self.northeast = None
        self.southwest = None
        self.southeast = None
    
    def subdivide(self):
        """Subdivide this quadtree into four quadrants"""
        x = self.boundary.x
        y = self.boundary.y
        w = self.boundary.width / 2
        h = self.boundary.height / 2
        
        # Create four children
        nw = Rectangle(x, y, w, h)
        ne = Rectangle(x + w, y, w, h)
        sw = Rectangle(x, y + h, w, h)
        se = Rectangle(x + w, y + h, w, h)
        
        # Initialize children quadtrees
        self.northwest = QuadTree(nw, self.capacity, self.max_depth, self.depth + 1)
        self.northeast = QuadTree(ne, self.capacity, self.max_depth, self.depth + 1)
        self.southwest = QuadTree(sw, self.capacity, self.max_depth, self.depth + 1)
        self.southeast = QuadTree(se, self.capacity, self.max_depth, self.depth + 1)
        
        # Move existing points to children
        for point in self.points:
            self.insert_to_children(point)
        
        self.points = []  # Clear points from this node
        self.divided = True
    
    def insert(self, point):
        """Insert a point into the quadtree"""
        # If point is not in boundary, don't insert
        if not self.boundary.contains(point):
            return False
        
        # If there's space and we haven't divided, add the point here
        if len(self.points) < self.capacity and not self.divided and self.depth < self.max_depth:
            self.points.append(point)
            return True
        
        # Otherwise, subdivide if needed and add to appropriate child
        if not self.divided and self.depth < self.max_depth:
            self.subdivide()
        
        if self.divided:
            return self.insert_to_children(point)
        
        # If we've reached max depth, just add it here
        self.points.append(point)
        return True
    
    def insert_to_children(self, point):
        """Insert a point into the appropriate child quadtree"""
        if self.northwest.insert(point):
            return True
        if self.northeast.insert(point):
            return True
        if self.southwest.insert(point):
            return True
        if self.southeast.insert(point):
            return True
        return False
    
    def query(self, range_rect, found=None):
        """Find all points within a range"""
        if found is None:
            found = []
        
        # If range doesn't intersect boundary, return empty list
        if not self.boundary.intersects(range_rect):
            return found
        
        # Check points at this level
        for point in self.points:
            px, py = point.x, point.y
            if (px >= range_rect.x - point.radius and
                px <= range_rect.x + range_rect.width + point.radius and
                py >= range_rect.y - point.radius and
                py <= range_rect.y + range_rect.height + point.radius):
                found.append(point)
        
        # If this node is divided, check children
        if self.divided:
            self.northwest.query(range_rect, found)
            self.northeast.query(range_rect, found)
            self.southwest.query(range_rect, found)
            self.southeast.query(range_rect, found)
        
        return found
    
    def query_circle(self, center_x, center_y, radius, found=None):
        """Find all points within a circular range"""
        if found is None:
            found = []
        
        # Create a rectangle that encompasses the circle
        range_rect = Rectangle(center_x - radius, center_y - radius, radius * 2, radius * 2)
        
        # If range doesn't intersect boundary, return empty list
        if not self.boundary.intersects(range_rect):
            return found
        
        # Check points at this level
        for point in self.points:
            # Calculate distance from point to circle center
            dx = point.x - center_x
            dy = point.y - center_y
            distance = (dx * dx + dy * dy) ** 0.5
            
            # If point is within circle (plus point's radius for collision)
            if distance <= radius + point.radius:
                found.append(point)
        
        # If this node is divided, check children
        if self.divided:
            self.northwest.query_circle(center_x, center_y, radius, found)
            self.northeast.query_circle(center_x, center_y, radius, found)
            self.southwest.query_circle(center_x, center_y, radius, found)
            self.southeast.query_circle(center_x, center_y, radius, found)
        
        return found
    
    def clear(self):
        """Clear all points from the quadtree"""
        self.points = []
        if self.divided:
            self.northwest.clear()
            self.northeast.clear()
            self.southwest.clear()
            self.southeast.clear()
            self.divided = False
            self.northwest = None
            self.northeast = None
            self.southwest = None
            self.southeast = None
    
    def draw(self, screen, color=(200, 200, 200)):
        """Draw the quadtree structure for visualization"""
        # Draw this node's boundary
        pygame.draw.rect(screen, color, 
                        (self.boundary.x, self.boundary.y, 
                         self.boundary.width, self.boundary.height), 
                        1)
        
        # Draw children if divided
        if self.divided:
            self.northwest.draw(screen, color)
            self.northeast.draw(screen, color)
            self.southwest.draw(screen, color)
            self.southeast.draw(screen, color)