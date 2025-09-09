# Controle de um Sistema de Dois Tanques

## 📌 Objetivo do Projeto

O objetivo deste projeto é **projetar um controlador** para um sistema composto por dois tanques de água interconectados. A meta é **regular o nível de água** no segundo tanque (*h₂*) para um valor de referência desejado, manipulando a vazão de entrada (*u*) no primeiro tanque.

O sistema será **linearizado** em torno de um ponto de operação específico para facilitar o projeto de um controlador linear.

---

## ⚙️ Especificações e Requisitos de Projeto

O controlador deve atender aos seguintes critérios de desempenho ao operar próximo ao ponto de equilíbrio especificado e rastrear uma referência degrau.

![Tanks System](ref/image.png)

### 🔹 Ponto de Equilíbrio para Linearização
- **Altura no Tanque 1 (h₁):** 0,6 m  
- **Altura no Tanque 2 (h₂):** 0,3 m

### 🔹 Desempenho para um Degrau de Referência de 0,8 m
- **Sobressinal Máximo (MS):** < 5%  
- **Tempo de Assentamento (tₛ):** < 8 segundos  
- **Erro em Regime Permanente:** Zero para referências do tipo degrau

---

## 📊 Visão Geral do Sistema

O sistema de dois tanques é composto por:
1. **Tanque 1** – recebe uma vazão de entrada controlada (*u*).  
2. **Tanque 2** – recebe água do Tanque 1 e representa a variável controlada (*h₂*).  

O problema de controle busca garantir rastreamento preciso e rejeição de perturbações, atendendo a todos os requisitos de desempenho transitório e de regime permanente.

## 📐 Modelo Linearizado

As equações linearizadas em torno do ponto de equilíbrio são:

$$
h1' = 20 \cdot \Delta u - 0.081 \cdot \Delta h1 + 0.081 \cdot \Delta h2
$$

$$
h2' = 0.081 \cdot \Delta h1 - 0.162 \cdot \Delta h2
$$

Com:

* $\Delta u = u - u_{equilíbrio}$
* $\Delta h1 = h1 - h1_{equilíbrio}$
* $\Delta h2 = h2 - h2_{equilíbrio}$

---

## 📉 Função de Transferência

Aplicando a transformada de Laplace e isolando a função de transferência $G = \frac{\Delta H2}{\Delta U}$, obtém-se:

$$
G(s) = \frac{1.62}{s^2 + 0.243s + 0.006}
$$
