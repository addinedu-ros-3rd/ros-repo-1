import mysql.connector

from utils.custom_logger import Logger
from utils.config_util import get_config

from mysql.connector import pooling

config = get_config()

log = Logger(__name__)

class DB():
    
    def __init__(self):
        db_config = {
            'host': config['host'],
            'user': config['user'],
            'password': config['password'],
            'database': config['database'],
        }
        
        self.connection_pool = pooling.MySQLConnectionPool(pool_name="my_pool", pool_size=5, **db_config)
        
        
    def execute(self, query, params=None):
        connection = self.connection_pool.get_connection()

        try:
            if connection.is_connected():
                cursor = connection.cursor(buffered=True)
                cursor.execute(query, params)
                connection.commit()

        except mysql.connector.Error as err:
            print(f"execute Error: {err}")
            
        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()
            

    def executeAndFetchAll(self, query, params=None):
        connection = self.connection_pool.get_connection()

        try:
            if connection.is_connected():
                cursor = connection.cursor(buffered=True)
                cursor.execute(query, params)
                connection.commit()
                
                result = cursor.fetchall()

                return result

        except mysql.connector.Error as err:
            print(f"executeAndFetchAll Error: {err}")
            
        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()


    def executeAndFetchOne(self, query, params=None):
        connection = self.connection_pool.get_connection()

        try:
            if connection.is_connected():
                cursor = connection.cursor(buffered=True)
                cursor.execute(query, params)
                connection.commit()
                
                result = cursor.fetchone()
                
                if result:
                    return result[0]
                else:
                    return 0

        except mysql.connector.Error as err:
            print(f"executeAndFetchOne Error: {err}")
            
        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()
                
                
    def callProc(self, proc_name, params):
        connection = self.connection_pool.get_connection()

        try:
            if connection.is_connected():
                cursor = connection.cursor(buffered=True)
                cursor.callproc(proc_name, params)
                connection.commit()

        except mysql.connector.Error as err:
            print(f"callProc Error: {err}")
            
        finally:
            if connection.is_connected():
                cursor.close()
                connection.close()