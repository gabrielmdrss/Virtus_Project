# Sistema Embarcado para Emulação de Funcionalidades de um Drone

Este projeto consiste em um sistema embarcado que simula funcionalidades essenciais de um drone, com foco em cálculo de posicionamento angular e detecção de colisão iminente. Utilizando o microcontrolador STM32F446RE e periféricos como o display SSD1306, o sensor MPU6050 e o sensor ultrassônico HC-SR04, o sistema oferece uma interface interativa para visualização e controle.

---

## ⚙️ **Funcionalidades Implementadas**

### **1. Sistema de Colisão Iminente (Ultrasonic)**
- Mede continuamente a distância até objetos utilizando o sensor ultrassônico HC-SR04.
- Permite ao usuário configurar o limite de segurança (5 a 200 cm) através de um potenciômetro.
- Alerta sobre colisões iminentes ativando um buzzer quando a distância medida for menor que o limite configurado.
- Uso de filtros passa-baixa para suavizar leituras do potenciômetro e do sensor.

### **2. Monitoramento do Movimento Angular (Gyroscope)**
- Visualiza as taxas de rotação nos eixos X, Y e Z do sensor MPU6050.
- Opera dentro da faixa de ±250°/s, exibindo dados no display SSD1306.
- Necessita de calibração inicial para compensar offsets nos registradores.

### **3. Cálculo do Posicionamento Angular (Kalman Angle)**
- Calcula os ângulos Pitch e Roll utilizando dados do acelerômetro e giroscópio.
- Implementa um filtro de Kalman unidimensional para fusão de dados, proporcionando estimativas estáveis e confiáveis.
- Exibe os ângulos estimados em tempo real no display.

### **4. Calibração do Sistema (Calibration)**
- Corrige desvios e offsets do MPU6050 para assegurar medições precisas.
- Executa 1000 amostras em 15 segundos para calcular o erro médio em cada eixo.
- Aplica os valores corrigidos diretamente nos registradores do sensor.

---

## 🛠️ **Hardware Utilizado**
- **Microcontrolador:** STM32F446RE (NUCLEO-F446RE).
- **Display:** OLED SSD1306 (I2C).
- **Sensores:**
  - **Inercial:** MPU6050 (acelerômetro e giroscópio de 6 eixos).
  - **Ultrassônico:** HC-SR04.
- **Outros Componentes:**
  - Buzzer.
  - 2 push-buttons.
  - Potenciômetro de 10kΩ.
---

## 🖥️ **Implementação do Software**
O software foi desenvolvido na **STM32CubeIDE** e estruturado para modularidade e integração eficiente. Algumas características técnicas incluem:
- **Configurações do MPU6050:**
  - Filtro passa-baixa de 20 Hz.
  - Faixa do acelerômetro: ±2g.
  - Faixa do giroscópio: ±250°/s.
  - Taxa de amostragem: 100 Hz (10 ms por amostra).
- **Interface de Usuário:**
  - Navegação no menu com dois push-buttons: `CHANGE` para alternar entre opções e `ENTER` para selecionar.
  - Controle implementado com detecção de borda para evitar duplicação de ícones.
- **Drivers Utilizados:**
  - Biblioteca externa para o display SSD1306.

---

## ✅ **Validações e Testes**
Os testes foram realizados de forma funcional exploratória, avaliando cada funcionalidade em cenários práticos.

1. **Sistema de Colisão Iminente:**
   - Configuração do limite de distância via potenciômetro, suavizado por filtro passa-baixa.
   - Testes demonstraram uma oscilação máxima de 1 cm, com acionamento confiável do buzzer ao cruzar o limite.

2. **Monitoramento Angular:**
   - Leituras dos eixos do giroscópio apresentaram consistência, mas offsets iniciais foram corrigidos após calibração.

3. **Cálculo de Ângulos (Kalman):**
   - Ângulos Pitch e Roll foram estimados com alta fidelidade, demonstrando estabilidade em comparação com os sensores individuais.

4. **Calibração:**
   - Offsets médios foram calculados e aplicados, resultando em medições mais precisas.

---
