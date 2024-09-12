## Aplicações Matemáticas em Robótica

Em robótica, várias aplicações da matemática podem ser identificadas no código fornecido. Aqui estão algumas que são relevantes para o funcionamento do robô:

### 1. Cinemática
A cinemática é a parte da mecânica que descreve o movimento sem considerar as forças que o causam. No código, o robô define as velocidades das rodas esquerda e direita com a função `set_velocities()`, o que está diretamente relacionado à cinemática diferencial.

A cinemática diferencial de robôs móveis de rodas considera a relação entre as velocidades das rodas e a trajetória do robô. A diferença entre a velocidade da roda esquerda e da direita altera o raio de curvatura da trajetória, influenciando a direção do movimento.

Exemplo matemático: Para um robô com rodas, a fórmula para calcular o movimento em linha reta seria:

v = (vL + vR) / 2

onde `v` é a velocidade linear do robô, `vL` e `vR` são as velocidades das rodas esquerda e direita, respectivamente.

### 2. Cálculo da Posição (Integração Numérica)
A função `get_position()` recupera a posição atual do robô no simulador. Se fosse um sistema real, seria necessário usar métodos de integração para calcular a posição com base nas velocidades das rodas e no tempo de deslocamento.

Se o robô se move com uma velocidade constante e por um intervalo de tempo `Δt`, a nova posição poderia ser calculada como:

x(t + Δt) = x(t) + vx * Δt

onde `vx` é a componente de velocidade na direção `x`. Isso envolve a aplicação de conceitos de cálculo integral.

### 3. Trigonometria
Se o robô se move em trajetórias curvas (o que ocorre quando as rodas têm velocidades diferentes), seria necessário utilizar trigonometria para determinar o ângulo de rotação e a nova posição em relação à direção original. Esse cálculo pode ser feito utilizando funções trigonométricas como seno e cosseno.

Para calcular a nova orientação `θ` do robô com base na diferença de velocidades das rodas:

Δθ = (vR - vL) / L * Δt


onde `L` é a distância entre as rodas.

### 4. Controle de Velocidade
O controle da velocidade das rodas é essencial para determinar a trajetória e o comportamento do robô. O código usa diretamente as velocidades das rodas para controlar o movimento. A matemática por trás desse controle pode envolver proporção entre as velocidades das rodas para manter uma trajetória reta ou girar em torno de um ponto.

### 5. Geometria Analítica
A geometria analítica pode ser usada para calcular a posição final do robô com base em suas coordenadas `x`, `y` e no ângulo `θ` que ele forma com a origem. A função `get_position()` retorna a posição `x`, `y`, `z`, que podem ser usadas em fórmulas geométricas para determinar o deslocamento em um plano 2D ou 3D.

---

Essas são algumas das aplicações matemáticas diretamente relacionadas ao comportamento descrito no código, mostrando como a matemática é aplicada no controle e no movimento de robôs móveis.

Para vizualizar a apresentação e o passo a passo para implementação do Sparki no CoppeliaSim, acesse: https://view.genially.com/66d9e2f7c3dd82d206514f65/interactive-image-sparki-no-coppelia
