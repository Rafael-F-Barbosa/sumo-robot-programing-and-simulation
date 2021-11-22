# Projeto de simulação de sumô de robos

## Resumo
Este projeto foi desenvolvido em conjunto por Rafael Fernandes Barbosa(@Rafael-F-Barbosa) e Marina Pinho Garcia(@ninapgarcia) para a disciplina de Processamento em Tempo Real - UnB no semestre 2021-1. O objetivo era desenvolver um algoritmo de de controle de um carrinho de sumô de robôs e realizar a simulação deste com o uso do software CoppeliamSim. Para isso foi desenvolvido um algoritmo simples, e foram definidos requisitos de tempo real que o programa deveria respeita

## Funcionamento

![RoboVsCubo1](/ProgramRunnningGifs/RoboVsCubo1.gif)

![RoboVsCubo2](/ProgramRunnningGifs/RoboVsCubo3.gif)

![RoboVsRobo1](/ProgramRunnningGifs/RoboVsRobo1.gif)

## Organização dos arquivos
Neste programa estão contidas as seguintes pastas com os seguintes arquivos:
- ThreadsCode: contém a implementação de dois programas um interruption handler (lê sensores e faz requisições) e um Controller que recebe as requisições e envia os comandos ao robô.
- ThreadsCodeRobot2: mesmo programa que o anterior, mas para a um segundo robô quando são simulados 2 ao mesmo tempo
- Measurements: Programa para medição dos tempos de resposta das tarefas implementadas pelos robôs.
- PyBindingLinux e PyBindingMac: programas disponibilizados pelo Coppelia para realizar a ligação entre os programas em python e o simulador em ambos os sistemas operacionais.
- Scenes: são cenas criadas no Coppelia para mostrar o funcionamento dos robõs

## Compilação e simulação

- Para testar os programas é necessário primeiro iniciar as cenas desejadas e então rodar o controlador (funciona como um servidor) e em seguida iniciar o Interruption Handler (que funciona como um cliente).
- No caso dos dois robôs é necessários rodar simultaneamente 4 programas, dois controladores e 2 clientes além de rodar a cena correspondente.