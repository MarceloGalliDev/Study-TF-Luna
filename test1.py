# main.py - Código para carregar no ESP32
from machine import I2C, Pin
import time

class VL6180X:
    """Driver para sensor VL6180X otimizado para ESP32"""
    
    # Registradores essenciais
    REG_IDENTIFICATION_MODEL_ID = 0x000
    REG_SYSTEM_INTERRUPT_CLEAR = 0x015
    REG_SYSTEM_FRESH_OUT_OF_RESET = 0x016
    REG_SYSRANGE_START = 0x018
    REG_RESULT_RANGE_STATUS = 0x04D
    REG_RESULT_RANGE_VAL = 0x062
    
    def __init__(self, sda_pin=21, scl_pin=22, address=0x29):
        """
        Inicializa o sensor VL6180X
        
        Args:
            sda_pin: Pino SDA (padrão GPIO 21)
            scl_pin: Pino SCL (padrão GPIO 22)
            address: Endereço I2C do sensor (padrão 0x29)
        """
        self.address = address
        
        print(f"Inicializando I2C nos pinos SDA={sda_pin}, SCL={scl_pin}")
        
        # Inicializa I2C
        self.i2c = I2C(0, scl=Pin(scl_pin), sda=Pin(sda_pin), freq=400000)
        
        # Verifica dispositivos I2C disponíveis
        devices = self.i2c.scan()
        print(f"Dispositivos I2C encontrados: {[hex(d) for d in devices]}")
        
        if self.address not in devices:
            raise RuntimeError(f"Sensor VL6180X não encontrado no endereço {hex(self.address)}")
        
        # Verifica modelo do sensor
        model_id = self._read_byte(self.REG_IDENTIFICATION_MODEL_ID)
        if model_id != 0xB4:
            raise RuntimeError(f"Modelo inválido! Esperado: 0xB4, Encontrado: {hex(model_id)}")
        
        # Inicializa sensor se necessário
        self._initialize_sensor()
        
        print("✅ Sensor VL6180X inicializado com sucesso!")
    
    def _read_byte(self, register):
        """Lê um byte do registrador (16-bit address)"""
        reg_bytes = [(register >> 8) & 0xFF, register & 0xFF]
        self.i2c.writeto(self.address, bytes(reg_bytes))
        return self.i2c.readfrom(self.address, 1)[0]
    
    def _write_byte(self, register, value):
        """Escreve um byte no registrador (16-bit address)"""
        reg_bytes = [(register >> 8) & 0xFF, register & 0xFF, value]
        self.i2c.writeto(self.address, bytes(reg_bytes))
    
    def _initialize_sensor(self):
        """Configurações iniciais obrigatórias"""
        if self._read_byte(self.REG_SYSTEM_FRESH_OUT_OF_RESET) == 1:
            print("Configurando sensor pela primeira vez...")
            
            # Configurações essenciais do datasheet
            config_registers = [
                (0x0207, 0x01), (0x0208, 0x01), (0x0096, 0x00), (0x0097, 0xfd),
                (0x00e3, 0x00), (0x00e4, 0x04), (0x00e5, 0x02), (0x00e6, 0x01),
                (0x00e7, 0x03), (0x00f5, 0x02), (0x00d9, 0x05), (0x00db, 0xce),
                (0x00dc, 0x03), (0x00dd, 0xf8), (0x009f, 0x00), (0x00a3, 0x3c),
                (0x00b7, 0x00), (0x00bb, 0x3c), (0x00b2, 0x09), (0x00ca, 0x09),
                (0x0198, 0x01), (0x01b0, 0x17), (0x01ad, 0x00), (0x00ff, 0x05),
                (0x0100, 0x05), (0x0199, 0x05), (0x01a6, 0x1b), (0x01ac, 0x3e),
                (0x01a7, 0x1f), (0x0030, 0x00)
            ]
            
            for reg, val in config_registers:
                self._write_byte(reg, val)
                time.sleep_ms(1)
            
            # Marca como inicializado
            self._write_byte(self.REG_SYSTEM_FRESH_OUT_OF_RESET, 0)
            print("Configuração inicial concluída")
    
    def read_distance(self):
        """
        Realiza uma medição de distância
        
        Returns:
            Distância em mm ou None se houver erro
        """
        try:
            # Inicia medição single-shot
            self._write_byte(self.REG_SYSRANGE_START, 0x01)
            
            # Aguarda medição completar (máximo 100ms)
            for _ in range(100):
                status = self._read_byte(self.REG_RESULT_RANGE_STATUS)
                if (status & 0x01) == 0x01:  # Nova amostra pronta
                    break
                time.sleep_ms(1)
            else:
                print("❌ Timeout na medição")
                return None
            
            # Verifica erro
            error_code = (status >> 4) & 0x0F
            if error_code != 0:
                error_msgs = {
                    1: "VCSEL Continuity Test",
                    2: "VCSEL Watchdog Test",
                    3: "VCSEL Watchdog",
                    4: "PLL1 Lock",
                    5: "PLL2 Lock",
                    6: "Early Convergence Estimate",
                    7: "Max Convergence",
                    8: "No Target Ignore",
                    9: "Max Signal To Noise Ratio",
                    10: "Raw Ranging Algo Underflow",
                    11: "Raw Ranging Algo Overflow",
                    12: "Ranging Algo Underflow",
                    13: "Ranging Algo Overflow"
                }
                print(f"❌ Erro: {error_msgs.get(error_code, f'Código {error_code}')}")
                return None
            
            # Lê distância
            distance = self._read_byte(self.REG_RESULT_RANGE_VAL)
            
            # Limpa flags de interrupção
            self._write_byte(self.REG_SYSTEM_INTERRUPT_CLEAR, 0x07)
            
            return distance
            
        except Exception as e:
            print(f"❌ Exceção na leitura: {e}")
            return None

def main():
    """Função principal"""
    try:
        print("=== ESP32 VL6180X Sensor ===")
        print("Aguarde inicialização...")
        
        # Cria instância do sensor
        sensor = VL6180X(sda_pin=21, scl_pin=22)
        
        print("\nIniciando leituras contínuas...")
        print("Pressione Ctrl+C para parar\n")
        
        reading_count = 0
        
        while True:
            distance = sensor.read_distance()
            reading_count += 1
            
            if distance is not None:
                # Formatação com emojis para melhor visualização
                if distance < 50:
                    status = "🔴 MUITO PRÓXIMO"
                elif distance < 100:
                    status = "🟡 PRÓXIMO"
                elif distance < 200:
                    status = "🟢 NORMAL"
                else:
                    status = "🔵 LONGE"
                
                print(f"[{reading_count:04d}] 📏 {distance:3d}mm | {status}")
            else:
                print(f"[{reading_count:04d}] ❌ ERRO NA LEITURA")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Leituras interrompidas pelo usuário")
        print("Obrigado!")
        
    except Exception as e:
        print(f"\n⚠️ ERRO CRÍTICO: {e}")
        print("\nVerifique:")
        print("- Conexões I2C (SDA=GPIO21, SCL=GPIO22)")
        print("- Alimentação 3.3V do sensor")
        print("- Endereço I2C (padrão 0x29)")

# Executar programa
if __name__ == "__main__":
    main()