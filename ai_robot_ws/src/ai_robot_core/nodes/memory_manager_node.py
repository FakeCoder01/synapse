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
        # This is a stub. A real implementation would use face recognition to identify the person.
        response.person_data = Person()
        response.person_data.is_known = False
        response.person_data.name = "Unknown"
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
