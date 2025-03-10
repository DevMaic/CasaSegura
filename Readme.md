<h1>
  <p align="center" width="100%">
    <img width="30%" src="https://softex.br/wp-content/uploads/2024/09/EmbarcaTech_logo_Azul-1030x428.png">
  </p>
</h1>

# ✨Tecnologias
Esse projeto foi desenvolvido com as seguintes tecnologias.
- Placa Raspberry Pi Pico W
- Raspberry Pi Pico SDK
- WokWi
- C/C++

# 💻Projeto
Projeto Desenvolvido durante o programa de capacitação profissional em microcontrolados e sistemas embarcados para estudantes de nível superior, polo Juazeiro-BA, na Universidade Federal do Vale do São Francisco (UNIVASF).

# 🚀Como rodar
### **Softwares Necessários**
1. **VS Code** com a extensão **Raspberry Pi Pico** instalada.
2. **CMake** e **Ninja** configurados.
3. **SDK do Raspberry Pi Pico** corretamente configurado.

### **Clonando o Repositório**
Para começar, clone o repositório no seu computador:
```bash
git clone https://github.com/DevMaic/CasaSegura
cd CasaSegura
```
---


### **Execução na Placa BitDogLab**
#### **1. Substituir o nome da sua rede wi-fi e senha no arquivo CasaSegura.c**
1. Localize nos cabeçalhos o WIFI_SSID e WIFI_PASS, e os substitua.
2. É de extrema importância que o nome da rede wi-fi e a senha estejam certos, o projeto não rodará se esse passo não for cumprido.
3. Para testes rápidos, é possível criar uma rede wifi até mesmo com um smartphone.
#### **2. Coloque em Modo Reboot**
1. Aperte o botão **BOOTSEL** no microcontrolador Raspberry Pi Pico W.
2. Ao mesmo tempo, aperte o botão de **Reset**.
#### **3. Upload de Arquivo `CasaSegura.uf2`**
1. Abra a pasta `build` incluída no repositório.
2. Mova o arquivo `CasaSegura.uf2` para a placa de desenvolvimento.
#### **4. Acompanhar Execução do Programa**
1. Aperte o botão A para simular o acionamento de um sensor, isso ligará a sirene e o led azul.
2. Aperte o botão B para simular o desligamento do alarme pelo usuário.
   
---
