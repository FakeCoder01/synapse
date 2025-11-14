import rclpy
from rclpy.node import Node
import sqlite3
import os
from ai_robot_msgs.msg import Person
from ai_robot_msgs.srv import LookupPerson, RegisterPerson, GetConversation, StoreConversation
from ament_index_python.packages import get_package_share_directory
import datetime
import uuid
import numpy as np

class MemoryManagerNode(Node):
    def __init__(self):
        super().__init__('memory_manager_node')
        db_path = os.path.join(os.getcwd(), 'robot_memory.db')
        self.conn = sqlite3.connect(db_path)
        self.create_tables()

        self.lookup_person_service = self.create_service(LookupPerson, '/lookup_person', self.lookup_person_callback)
        self.register_person_service = self.create_service(RegisterPerson, '/register_person', self.register_person_callback)
        self.get_conversation_service = self.create_service(GetConversation, '/get_conversation', self.get_conversation_callback)
        self.store_conversation_service = self.create_service(StoreConversation, '/store_conversation', self.store_conversation_callback)

    def create_tables(self):
        cursor = self.conn.cursor()
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS people (
            person_id TEXT PRIMARY KEY,
            name TEXT,
            first_seen TIMESTAMP,
            face_encoding_stub BLOB
        )
        """)
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS conversations (
            convo_id INTEGER PRIMARY KEY,
            person_id TEXT,
            timestamp TIMESTAMP,
            speaker TEXT,
            line TEXT
        )
        """)
        self.conn.commit()

    def lookup_person_callback(self, request, response):
        """Looks up a person by comparing face encodings."""
        response.person_data = Person()
        response.person_data.is_known = False
        response.person_data.name = "Unknown"

        try:
            request_encoding = np.array(request.face_encoding, dtype=np.float32)
            
            cursor = self.conn.cursor()
            cursor.execute("SELECT person_id, name, face_encoding_stub FROM people")
            all_people = cursor.fetchall()

            best_match_id = None
            best_match_name = None
            min_distance = 0.6 # Confidence threshold; lower is more strict.

            for person_id, name, encoding_blob in all_people:
                known_encoding = np.frombuffer(encoding_blob, dtype=np.float32)
                
                # Calculate Euclidean distance (L2 norm)
                distance = np.linalg.norm(request_encoding - known_encoding)
                
                if distance < min_distance:
                    min_distance = distance
                    best_match_id = person_id
                    best_match_name = name

            if best_match_id:
                response.person_data.is_known = True
                response.person_data.person_id = best_match_id
                response.person_data.name = best_match_name
                self.get_logger().info(f"Found match: {best_match_name} with distance {min_distance}")

        except Exception as e:
            self.get_logger().error(f"Error during person lookup: {e}")
            
        return response

    def register_person_callback(self, request, response):
        person_id = str(uuid.uuid4())
        cursor = self.conn.cursor()
        encoding = np.array(request.face_encoding).tobytes()
        cursor.execute("INSERT INTO people (person_id, name, first_seen, face_encoding_stub) VALUES (?, ?, ?, ?)",
                       (person_id, request.name_hint, datetime.datetime.now(), encoding))
        self.conn.commit()
        response.person_id = person_id
        return response

    def get_conversation_callback(self, request, response):
        cursor = self.conn.cursor()
        cursor.execute("SELECT speaker, line FROM conversations WHERE person_id = ? ORDER BY timestamp DESC LIMIT ?", (request.person_id, request.line_count))
        rows = cursor.fetchall()
        response.history = [f"{row[0]}: {row[1]}" for row in rows]
        return response

    def store_conversation_callback(self, request, response):
        cursor = self.conn.cursor()
        cursor.execute("INSERT INTO conversations (person_id, timestamp, speaker, line) VALUES (?, ?, ?, ?)",
                       (request.person_id, datetime.datetime.now(), request.speaker, request.line))
        self.conn.commit()
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MemoryManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
