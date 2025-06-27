# flake8: noqa
# pylint: disable=all

import serial
import time
import csv
from datetime import datetime
import threading
import queue

class TFLunaReader:
    def __init__(self, port='COM3', baudrate=115200, timeout=1):
        """
        Inicializa o leitor do sensor TF-Luna via ESP32
        
        Args:
            port (str): Porta serial (ex: 'COM3' no Windows, '/dev/ttyUSB0' no Linux)
            baudrate (int): Taxa de transmissão (115200 conforme ESP32)
            timeout (float): Timeout para leitura serial
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.is_running = False
        self.data_queue = queue.Queue()
        
    def connect(self):
        """Conecta à porta serial"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print(f"Conectado à porta {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Erro ao conectar: {e}")
            return False
    
    def disconnect(self):
        """Desconecta da porta serial"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Desconectado da porta serial")
    
    def read_distance(self):
        """
        Lê uma medição de distância do sensor
        Retorna a distância em cm ou None se erro
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        try:
            # Limpa buffer de entrada
            self.serial_conn.flushInput()
            
            # Lê linha da serial (formato: "Distância: XXX cm")
            line = self.serial_conn.readline().decode('utf-8').strip()
            
            if "Distância:" in line:
                # Extrai o valor numérico (agora em mm)
                distance_str = line.split("Distância:")[1].split("mm")[0].strip()
                distance = int(distance_str)
                return distance
            
        except (serial.SerialException, ValueError, UnicodeDecodeError) as e:
            print(f"Erro na leitura: {e}")
            
        return None
    
    def start_continuous_reading(self, callback=None):
        """
        Inicia leitura contínua em thread separada
        
        Args:
            callback: Função chamada a cada leitura callback(distance, timestamp)
        """
        self.is_running = True
        
        def read_loop():
            while self.is_running:
                distance = self.read_distance()
                if distance is not None:
                    timestamp = datetime.now()
                    
                    # Adiciona à fila
                    self.data_queue.put((distance, timestamp))
                    
                    # Chama callback se fornecido
                    if callback:
                        callback(distance, timestamp)
                
                time.sleep(0.1)  # Pequena pausa entre leituras
        
        # Inicia thread de leitura
        self.read_thread = threading.Thread(target=read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        print("Leitura contínua iniciada")
    
    def stop_continuous_reading(self):
        """Para a leitura contínua"""
        self.is_running = False
        print("Leitura contínua parada")
    
    def get_data_from_queue(self):
        """Retorna todos os dados disponíveis na fila"""
        data = []
        while not self.data_queue.empty():
            try:
                data.append(self.data_queue.get_nowait())
            except queue.Empty:
                break
        return data
    
    def save_to_csv(self, filename, duration_seconds=30):
        """
        Salva dados em CSV por período determinado
        
        Args:
            filename (str): Nome do arquivo CSV
            duration_seconds (int): Duração da coleta em segundos
        """
        print(f"Coletando dados por {duration_seconds} segundos...")
        
        with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Timestamp', 'Distancia_mm'])
            
            start_time = time.time()
            
            while time.time() - start_time < duration_seconds:
                distance = self.read_distance()
                if distance is not None:
                    timestamp = datetime.now()
                    writer.writerow([timestamp.strftime('%Y-%m-%d %H:%M:%S.%f'), distance])
                    print(f"{timestamp.strftime('%H:%M:%S')} - Distância: {distance} mm")
                
                time.sleep(0.5)  # Leitura a cada 500ms
        
        print(f"Dados salvos em {filename}")

def main():
    # Exemplo de uso
    
    # Configurar porta (ajustar conforme seu sistema)
    # Windows: 'COM3', 'COM4', etc.
    # Linux/Mac: '/dev/ttyUSB0', '/dev/ttyACM0', etc.
    PORTA_SERIAL = 'COM3'  # Altere conforme necessário
    
    # Criar instância do leitor
    sensor = TFLunaReader(port=PORTA_SERIAL)
    
    # Conectar
    if not sensor.connect():
        print("Falha na conexão. Verifique a porta serial.")
        return
    
    try:
        print("Escolha uma opção:")
        print("1 - Leitura única")
        print("2 - Leitura contínua (pressione Ctrl+C para parar)")
        print("3 - Salvar dados em CSV")
        
        opcao = input("Opção: ").strip()
        
        if opcao == "1":
            # Leitura única
            print("Fazendo leitura única...")
            distance = sensor.read_distance()
            if distance:
                print(f"Distância medida: {distance} mm")
            else:
                print("Erro na leitura")
        
        elif opcao == "2":
            # Leitura contínua
            def print_reading(distance, timestamp):
                print(f"{timestamp.strftime('%H:%M:%S')} - Distância: {distance} mm")
            
            sensor.start_continuous_reading(callback=print_reading)
            
            try:
                while True:
                    time.sleep(1)
            except KeyboardInterrupt:
                sensor.stop_continuous_reading()
        
        elif opcao == "3":
            # Salvar em CSV
            filename = f"tf_luna_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            duration = int(input("Duração da coleta (segundos): "))
            sensor.save_to_csv(filename, duration)
        
        else:
            print("Opção inválida")
    
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário")
    
    finally:
        sensor.disconnect()

if __name__ == "__main__":
    main()