# F10-Instrument-Cluster-Simhub
F10 instrument cluster to work with Simhub for a game of your choice.

Project was made with Arduino UNO (China) and CAN BUS Shield V2 (China)

F10.ino file to test the speedo, adresses etc.

F10CustomProtocol.txt file for SimHub and Custom Protocol NCalc Formula
```
format([DataCorePlugin.GameData.NewData.SpeedKmh],'0') + ';' +
format([DataCorePlugin.GameData.NewData.Rpms],'0') + ';' +
format([DataCorePlugin.GameData.NewData.Gear],'0') + ';' +
format([DataCorePlugin.GameData.NewData.FuelPercent],'0') + ';' +
format([DataCorePlugin.GameData.NewData.EngineOilTemp],'0') + ';' +
[EngineIgnitionOn]
```
![ezgif-28b04f63af7ae4](https://github.com/user-attachments/assets/31d736cc-24fd-4506-a8b6-097b3a89a8ca)
