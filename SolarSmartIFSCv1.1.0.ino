/*Projeto: Comunicação Serial Solar/Sm@rtIFSC
Tipo: RS485
Protocolo: Modbus
Versão Código: 1.1.0*/
/*Obs:
--> variáveis que iniciam com sf_ são correspondem aos fatores de escala
das medidas. Variáveis que finalizam com _sf correspondem às variáveis com
fator de escala presente na variável;
--> Esta versão comporta salvamento
dos dados localmente.*/

#include<SPI.h>
#include<Ethernet.h>
#include<ModbusMaster.h>
#include<SD.h>

//------------------------> definição dos pinos <--------------------
#define rxPin 2
#define txPin 3
#define MAX485_DE 5
#define MOSI 11
#define MISO 12
#define CLK 13
#define CS 4
//------------------------------> end <------------------------------

#define baud 9600
#define DEBUG

//Estrutura de dados coletados dos inversores
typedef struct{
    uint16_t id_device;
    uint16_t op_mode;
    uint16_t vendor_op_mode;
    uint16_t tot_accurrent;
    uint16_t phase_current[3];
    int16_t  sf_accurrent;
    uint16_t voltage_ph_ph[3];
    uint16_t voltage_ph_nt[3];
    int16_t  sf_voltage;
    int16_t  ac_infeed_sf[2];
    uint16_t ac_freq;
    int16_t  sf_ac_freq;
    int16_t  powergen_sf[6];
    uint32_t tot_fedin_energy[2];
    int16_t  sf_tot_fedin;
    uint16_t dc_current;
    int16_t  sf_dc_current;
    uint16_t dc_voltage;
    int16_t  sf_dc_voltage;
    int16_t  dc_pwgen_stat_sf[7];
  } RegistersData;

//endereço dos escravos
const uint8_t slavesAddr[] = {0x01, 0x02, 0x03, 0x04};
//tamanho do endereço
const uint8_t slaveslen = sizeof(slavesAddr) / sizeof(slavesAddr[0]);
char serverIp[] = "172.16.0.179";
//resposta modbus
uint8_t result;

//==================================================================
        ModbusMaster node;
        File debug;//arq backup fixo
        File dbtemp;//arq de envio temporário
        bool SendDataToServer(RegistersData *data, EthernetClient *server);
        void SaveDataToLocal(RegistersData *data);
        void SaveDataTemp(RegistersData *data);
        bool SetslaveResetCommands(uint16_t command);
        bool getSlaveRegisters(RegistersData *data);
//==================================================================


//==================================================================
//rotinas pré e pós transmissão executadas a cada frame enviado

void preTransmission(){
  digitalWrite(MAX485_DE, HIGH);
}

void postTransmission(){
  delay(10);
  digitalWrite(MAX485_DE, LOW);
  delay(50);
}
//=================================================================
void setup() {

  if(!SD.begin(4)){
  Serial.println("Inicialização falhou!");
 // while(1); ?????<--------
  }
  Serial.println("Inicialização concluída.");

  pinMode(MAX485_DE, OUTPUT);

        //iniciar em modo leitura
        digitalWrite(MAX485_DE, LOW);

        //setar baud para saídas seriais
        Serial2.begin(baud);
        Serial.begin(baud);
        //setar pinos de entrada e saída
        pinMode(rxPin, INPUT);
        pinMode(txPin, OUTPUT);

//system("stty -F /dev/ttyS1 9600 cs8 -cstopb -parenb");

debug = SD.open("/smartifsc/solar/debugsolar.log", FILE_WRITE);

//configuração do transceptor rs485
node.preTransmission(preTransmission);
node.postTransmission(postTransmission);
}
//===================================================================

//====================== > xxxxxxxxxxxx < ===========================
void loop() {

dbtemp = SD.open("/smartifsc/solar/dbtemp.log",FILE_WRITE);

      uint8_t s = 0;
      static RegistersData registers;
      static EthernetClient client;

      while(s < slaveslen){

          node.begin(slavesAddr[s], Serial2);
          if (getSlaveRegisters(&registers)){
            //dado recebido!!!
            #ifdef DEBUG
            debug.print("\ndata RECEIVED from slave:" );
            debug.print(slavesAddr[s]);
            debug.flush();
            #endif
                  //salvando arquivos no backup fixo
                  SaveDataToLocal(&registers);

                  //salvando dados no arquivo de envio
                  SaveDataTemp(&registers);
          }
          else{//dado não recebido
          #ifdef DEBUG
          debug.print("\nData not received from slave: " );
          debug.print(slavesAddr[s]);
          debug.flush();
          #endif
          }
        s++;
      }

      if(!client.connected()){
        client.connect(serverIp, 80);
      }
      if(sendDataToServer(&registers, &client)){
        //dados enviados ao servidor
      }
      else{//dados não enviados!!!
        #ifdef DEBUG
        debug.print("\nData not sent to server. \n");
        debug.flush();
        #endif
      }
}//end!!!
//-----------------> escopo das funções <-----------------------

bool getSlaveRegisters(RegistersData *data)
{
  bool ok = true;
  uint8_t i, sze;

//id_device acquisition - register 40069
  result = node.readHoldingRegisters(40069,40069);
  #ifdef DEBUG
  debug.print("\nFIRST RESULT: ");
  debug.print(result);
  debug.flush();
  #endif

  if (result == node.ku8MBSuccess){
    data->id_device = result;
  }
  else{
  #ifdef DEBUG
  debug.print("\n40069! ");
  debug.print(result);
  debug.flush();
  #endif
  }

//op_mode acquisition - register 40108
  result = node.readHoldingRegisters(40108,40108);
  if (result == node.ku8MBSuccess){
    data->op_mode = result;
  }
  else{
  ok = false;
  #ifdef DEBUG
  debug.print("\n40108! ");
  debug.print(result);
  debug.flush();
  #endif
  }

//vendor_op_mode acquisition - register 40109
  result = node.readHoldingRegisters(40109,40109);
  if (result == node.ku8MBSuccess){
    data->vendor_op_mode = result;
  }
  else{
    ok = false;
    #ifdef DEBUG
    debug.print("\n40109! ");
    debug.print(result);
    debug.flush();
    #endif
    }

//tot_accurrent - register 40072
  result = node.readHoldingRegisters(40072, 40072);
  if (result == node.ku8MBSuccess){
    data->tot_accurrent = result;
  }
  else{
    ok = false;
    #ifdef DEBUG
    debug.write("\n40072! ");
    debug.write(result);
    debug.flush();
    #endif
    }

//phase_current - registers 40073,40074 and 40075
    result = node.readHoldingRegisters(40073, 40075-40073);
    if (result == node.ku8MBSuccess){
      sze = sizeof(data->phase_current) / sizeof(data->phase_current[0]);
      for( i=0 ; i < sze ; i++ ){
          data->phase_current[i] = node.getResponseBuffer(i);
      }
    }
    else{
    ok = false;
    #ifdef DEBUG
    debug.write("\n40073! ");
    debug.write(result);
    debug.flush();
    #endif
    }

//sf_accurrent - register 40076
    result = node.readHoldingRegisters(40076, 40076);
    if(result == node.ku8MBSuccess){
      data-> sf_accurrent = result;
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40076! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//voltage_ph_ph - registers 40077,40078 and 40079
    result = node.readHoldingRegisters(40077,40079 - 40077);
    if(result == node.ku8MBSuccess){
      sze = sizeof(data->voltage_ph_ph) / sizeof(data->voltage_ph_ph[0]);
      for( i=0 ; i < sze ; i++){
        data->voltage_ph_ph[i] = node.getResponseBuffer(i);
      }
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40077! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//voltage_ph_nt - registers 40080,40081 and 40082
    result = node.readHoldingRegisters(40080, 40082 - 40080);
    if(result == node.ku8MBSuccess){
      sze = sizeof(data->voltage_ph_nt) / sizeof(data->voltage_ph_nt[0]);
      for( i=0 ; i < sze ; i++ ){
        data-> voltage_ph_nt[i] = node.getResponseBuffer(i);
      }
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40080! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//sf_voltage - register 40083
    result = node.readHoldingRegisters(40083, 40083);
    if(result == node.ku8MBSuccess){
      data-> sf_voltage = result;
      }
      else{
        ok = false;
        #ifdef DEBUG
        debug.write("\n40083! ");
        debug.write(result);
        debug.flush();
        #endif
      }

//ac_infeed_sf - registers 40084 and 40085
    result = node.readHoldingRegisters(40084,40085 - 40084);
    if( result == node.ku8MBSuccess){
      sze = sizeof(data->ac_infeed_sf) / sizeof(data->ac_infeed_sf[0]);
      for( i=0 ; i < sze ; i++ ){
        data-> ac_infeed_sf[i] = node.getResponseBuffer(i);
      }
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40084! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//ac_freq - register 40086
    result = node.readHoldingRegisters(40086, 40086);
    if(result == node.ku8MBSuccess){
      data->ac_freq = result;
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40086! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//sf_ac_freq - register 40087
    result = node.readHoldingRegisters(40087, 40087);
    if(result == node.ku8MBSuccess){
      data->sf_ac_freq = result;
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40087! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//powergen_sf - registers 40088,40089,40090,40091,40092 and 40093
    result = node.readHoldingRegisters(40088,40093 - 40088);
    if( result == node.ku8MBSuccess){
      sze = sizeof(data->powergen_sf) / sizeof(data->powergen_sf[0]);
      for( i=0 ; i < sze ; i++){
        data->powergen_sf[i] = node.getResponseBuffer(i);
      }
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40088! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//tot_fedin_energy - registers 40094 and 40095
    result = node.readHoldingRegisters(40094, 40095 - 40094);
    if(result == node.ku8MBSuccess){
      sze = sizeof(data->tot_fedin_energy)/sizeof(data->tot_fedin_energy[0]);
      for( i=0 ; i < sze ; i++){
      data->tot_fedin_energy[i] = node.getResponseBuffer(i);
      }
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40094! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//sf_tot_fedin - registers 40096
    result = node.readHoldingRegisters(40096, 40096);
    if(result == node.ku8MBSuccess){
      data->sf_tot_fedin = result;
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40096! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//dc_current - register 40097
    result = node.readHoldingRegisters(40097, 40097);
    if(result == node.ku8MBSuccess){
      data->dc_current = result;
      }
      else{
        ok = false;
        #ifdef DEBUG
        debug.write("\n40097! ");
        debug.write(result);
        debug.flush();
        #endif
      }

//sf_dc_current - register 40098
    result = node.readHoldingRegisters(40098, 40098);
    if(result == node.ku8MBSuccess){
      data->sf_dc_current = result;
      }
      else{
        ok = false;
        #ifdef DEBUG
        debug.write("\n40098! ");
        debug.write(result);
        debug.flush();
        #endif
      }

//dc_voltage - register 40099
    result = node.readHoldingRegisters(40099, 40099);
    if( result == node.ku8MBSuccess){
      data->dc_voltage = result;
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40099! ");
      debug.write(result);
      debug.flush();
      #endif
    }

//sf_dc_voltage - register 40100
    result = node.readHoldingRegisters(40100, 40100);
    if(result == node.ku8MBSuccess){
      data->sf_dc_voltage = result;
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40100! ");
      debug.write(result);
      debug.flush();
      #endif
    }

/*dc_pwgen_stat_sf - registers 40101,40102,40103,40104,40105
                               40105,40106 and 40107*/
    result = node.readHoldingRegisters(40101, 40107 - 40101);
    if(result == node.ku8MBSuccess){
      sze = sizeof(data->dc_pwgen_stat_sf)/sizeof(data->dc_pwgen_stat_sf[0]);
      for( i=0 ; i < sze ; i++){
        data->dc_pwgen_stat_sf[i] = node.getResponseBuffer(i);
      }
    }
    else{
      ok = false;
      #ifdef DEBUG
      debug.write("\n40101! ");
      debug.write(result);
      debug.flush();
      #endif
    }
return ok;
}

//Function SavedataToLocalsaving all registers in sdcard
void SaveDataToLocal(RegistersData *data) {

 uint8_t i, sze;  
  
  //saving id_device
  debug.print("\n-> id_device: ");
  debug.print(data->id_device,DEC);
  debug.flush();

  //saving op_mode quisition
  debug.print("\n-> op_mode: ");
  debug.print(data->op_mode,DEC);
  debug.flush();

  //saving vendor_op_mode aquisition
  data->vendor_op_mode;
  debug.print("\n-> vendor_op_mode: ");
  debug.print(data->vendor_op_mode,DEC);
  debug.flush();

  //saving tot_accurrent
  debug.print("\n-> tot_accurent: ");
  debug.print(data->tot_accurrent,DEC);
  debug.flush();

  //saving phase_current
  debug.print("\np -> phase_current A,B and C:");
  sze = sizeof(data->phase_current) / sizeof(data->phase_current[0]);
  for(i=0;i<sze;i++){
  debug.print("-> phase_current : ");
  debug.print(data->phase_current[i],DEC);
  debug.flush();
  }
  
  //saving sf_accurrent
  debug.print("\n-> sf_accurrent: ");
  debug.print(data->sf_accurrent,DEC);
  debug.flush();

  //saving voltage_ph_ph
  debug.print("\n-> voltage_ph_ph AB,BC and CA: ");
  sze = sizeof(data->voltage_ph_ph) / sizeof(data->voltage_ph_ph[0]);
  for(i=0;i<sze;i++){
  debug.print("\n-> voltage_ph_ph : ");
  debug.print(data->voltage_ph_ph[i],DEC);
  debug.flush();
  }

  //saving voltage_ph_NT
  debug.print("\n-> voltage_ph_nt AN,BN and CN: ");
  sze = sizeof(data->voltage_ph_nt) / sizeof(data->voltage_ph_nt[0]);
  for(i=0;i<sze;i++){
  debug.print("\n-> voltage_ph_nt : ");
  debug.print(data->voltage_ph_nt[i],DEC);
  debug.flush();
  }

  //saving sf_voltage
  debug.print("\n-> sf_voltage: ");
  debug.print(data->sf_voltage,DEC);
  debug.flush();

  //saving ac_infeed_sf
  debug.print("\n-> ac_infeed_sf: ");
  sze = sizeof(data->ac_infeed_sf) / sizeof(data->ac_infeed_sf[0]);
  for(i=0;i<sze;i++){
  debug.print("\n-> ac_infeed_sf : ");
  debug.print(data->ac_infeed_sf[i],DEC);
  debug.flush();
  }

  //saving ac_freq
  debug.print("\n-> ac_frequency:");
  debug.print(data->ac_freq,DEC);
  debug.flush();

  //saving sf_ac_freq
  debug.print("\n-> sf_ac_frequency:");
  debug.print(data->sf_ac_freq,DEC);
  debug.flush();

  //saving powergen_sf
  debug.print("\n-> powergen_sf: ");
  sze = sizeof(data->powergen_sf) / sizeof(data->powergen_sf[0]);
  for(i=0;i<sze;i++){
  debug.print("\n-> powergent_sf : ");
  debug.print(data->powergen_sf[i],DEC);
  debug.flush();
  }

  //saving tot_fedin_energy
  debug.print("\n-> tot_fedin_energy: ");
  sze = sizeof(data->tot_fedin_energy) / sizeof(data->tot_fedin_energy[0]);
  for(i=0;i<sze;i++){
  debug.print("\n-> tot_fedin_energy : ");
  debug.print(data->tot_fedin_energy[i],DEC);
  debug.flush();
  }

  //saving sf_tot_fedin_energy
  debug.print("\n-> sf_tot_fedin_energy: ");
  debug.print(data->sf_tot_fedin,DEC);
  debug.flush();
  
  //saving dc_current
  debug.print("\n-> dc_current: ");
  debug.print(data->dc_current,DEC);
  debug.flush();

  //saving sf_dc_current
  debug.print("\n-> sf_dc_current: ");
  debug.print(data->sf_dc_current,DEC);
  debug.flush();

  //saving dc_voltage
  debug.print("\n-> dc_voltage: ");
  debug.print(data->dc_voltage,DEC);
  debug.flush();

  //saving sf_dc_voltage
  debug.print("\n-> sf_dc_voltage: ");
  debug.print(data->sf_dc_voltage,DEC);
  debug.flush();

  //saving dc_pwgen_stat_sf
  debug.print("\n-> dc_pwgen_stat_sf: ");
  sze = sizeof(data->dc_pwgen_stat_sf) / sizeof(data->dc_pwgen_stat_sf[0]);
  for( i = 0 ; i < sze ; i++ );{
    debug.print("\n-> dc_pwgen_stat_sf: ");
    debug.print(data->dc_pwgen_stat_sf[i],DEC);
    debug.flush();
    }
 //END
  }

//Function SaveDataTemp
void SaveDataTemp(RegistersData *data){
  
  uint8_t i, sze;  
  
  //saving id_device
  //dbtemp.write("&id_device=");
  dbtemp.write(data->id_device);
  dbtemp.write(",");
  //dbtemp.write(data->id_device);
  dbtemp.flush();

  //saving op_mode quisition
  //dbtemp.write("&op_mode=");
  dbtemp.write(data->op_mode);
  dbtemp.write(",");
  //dbtemp.write(data->op_mode);
  dbtemp.flush();

  //saving vendor_op_mode aquisition
  //dbtemp.write("&vendor_op_mode=");
  dbtemp.write(data->vendor_op_mode);
  dbtemp.write(",");
  //dbtemp.write(data->vendor_op_mode);
  dbtemp.flush();
 
  //saving tot_accurrent
  //dbtemp.write("&tot_accurrent=");
  dbtemp.write(data->tot_accurrent);
  dbtemp.write(",");
  //dbtemp.write(tot_accurrent);
  dbtemp.flush();

  //saving phase_current
  //dbtemp.print("&phase_current=");
  sze = sizeof(data->phase_current) / sizeof(data->phase_current[0]);
  for(i=0;i<sze;i++){
  dbtemp.write(data->phase_current[i]);
  dbtemp.write(",");
  }
  //dbtemp.write(phase_current[i]);
  dbtemp.flush();
  
  //saving sf_accurrent
  //dbtemp.print("&sf_accurrent=");
  dbtemp.write(data->sf_accurrent);
  dbtemp.write(",");
  //dbtemp.write(data->sf_accurrent);
  dbtemp.flush();

  //saving voltage_ph_ph
  //dbtemp.print("&voltage_ph_ph=");
  sze = sizeof(data->voltage_ph_ph) / sizeof(data->voltage_ph_ph[0]);
  for(i=0;i<sze;i++){
  dbtemp.write(data->voltage_ph_ph[i]);
  dbtemp.write(",");
  }
  //dbtemp.write(voltage_ph_ph[i]);
  dbtemp.flush();
  

  //saving voltage_ph_nt
  //dbtemp.print("&voltage_ph_nt=");
  sze = sizeof(data->voltage_ph_nt) / sizeof(data->voltage_ph_nt[0]);
  for(i=0;i<sze;i++){
  dbtemp.write(data->voltage_ph_nt[i]);
  dbtemp.write(",");
  }
  //dbtemp.write(voltage_ph_nt[i]);
  dbtemp.flush();
  
  //saving sf_voltage
  //dbtemp.print("&sf_voltage=");
  dbtemp.write(data->sf_voltage);
  dbtemp.write(",");
  //dbtemp.write(data->sf_voltage);
  debug.flush();

  //saving ac_infeed_sf
  //dbtemp.print("&ac_infeed_sf=");
  sze = sizeof(data->ac_infeed_sf) / sizeof(data->ac_infeed_sf[0]);
  for(i=0;i<sze;i++){
  dbtemp.write(data->ac_infeed_sf[i]);
  dbtemp.write(",");
  }
  //dbtemp.write(data->ac_infeed_sf[i]);
  dbtemp.flush();

  //saving ac_freq
  //dbtemp.print("&ac_frequency=");
  dbtemp.write(data->ac_freq);
  dbtemp.write(",");
  //dbtemp.write(data->ac_freq);
  dbtemp.flush();

  //saving sf_ac_freq
  //dbtemp.print("&sf_ac_frequency=");
  dbtemp.write(data->sf_ac_freq);
  dbtemp.write(",");
  //dbtemp.write(data->sf_ac_freq);
  dbtemp.flush();

  //saving powergen_sf
  //dbtemp.print("&powergen_sf=");
  sze = sizeof(data->powergen_sf) / sizeof(data->powergen_sf[0]);
  for(i=0;i<sze;i++){
  dbtemp.write(data->powergen_sf[i]);
  dbtemp.write(",");
  }
  //dbtemp.write(data->powergen_sf[i]);
  dbtemp.flush();
  
  //saving tot_fedin_energy
  //dbtemp.print("&tot_fedin_energy=");
  sze = sizeof(data->tot_fedin_energy) / sizeof(data->tot_fedin_energy[0]);
  for(i=0;i<sze;i++){
  dbtemp.write(data->tot_fedin_energy[i]);
  dbtemp.write(",");
  }
  //dbtemp.write(data->tot_fedin_energy[i]);
  dbtemp.flush();
  
  //saving sf_tot_fedin_energy
  //dbtemp.print("&sf_tot_fedin_energy=");
  dbtemp.write(data->sf_tot_fedin);
  dbtemp.write(",");
  //dbtemp.write(data->sf_tot_fedin_energy);
  dbtemp.flush();
  
  //saving dc_current
  //dbtemp.print("&dc_current=");
  dbtemp.write(data->dc_current);
  dbtemp.write(",");
  //dbtemp.write(data->dc_current);
  dbtemp.flush();

  //saving sf_dc_current
  //dbtemp.print("&sf_dc_current=");
  dbtemp.write(data->sf_dc_current);
  dbtemp.write(",");
  //dbtemp.write(data->sf_dc_current);
  dbtemp.flush();

  //saving dc_voltage
  //dbtemp.print("&dc_voltage=");
  dbtemp.write(data->dc_voltage);
  dbtemp.print(",");
  //dbtemp.write(data->dc_voltage);
  dbtemp.flush();

  //saving sf_dc_voltage
  //dbtemp.print("&sf_dc_voltage=");
  dbtemp.write(data->sf_dc_voltage);
  dbtemp.write(",");
  //dbtemp.write(data->sf_dc_voltage);
  dbtemp.flush();
  
  //saving dc_pwgen_stat_sf
  //dbtemp.print("&dc_pwgen_stat_sf=");
  sze = sizeof(data->dc_pwgen_stat_sf) / sizeof(data->dc_pwgen_stat_sf[0]);
  for( i = 0 ; i < sze ; i++ );{
    dbtemp.write(data->dc_pwgen_stat_sf[i]);
    dbtemp.write(",");
    }  
    //dbtemp.write(data->dc_pwgen_stat_sf[i]);
    dbtemp.flush();
 
  }//END

//senddatatoserver método ninja

bool sendDataToServer(RegistersData *data, EthernetClient *client)
{

    bool sent = false;
    uint8_t i, sze, pos;
    if(client->connected()){
    client->print("GET http://");
    client->print(serverIp);
    client->print("/smartifsc/php/pm210.php");

    //sending file
    sze = dbtemp.size();
    pos = 0;
    dbtemp.seek(pos);
    for(i = 0 ; i < sze ; i++){
      client->print(dbtemp.peek());
      pos++;
      dbtemp.seek(pos);
      }  
    
    // info
    client->println("HTTP/1.1");
    client->println( "Host: smartifsc" );
    client->println( "Content-Type: application/x-www-form-urlencoded" );
    client->println( "Connection: close" );
    client->println();
    client->println();
    client->stop();
    client->flush();
    sent = true;
    }
    else{//cliente não conectado
      #ifdef DEBUG
      debug.write("client not connected. \n");
      debug.flush();
      #endif
            client->stop();
            client->flush();
            sent = false;

      }
      return sent;
}


/*bool sendDataToServer(RegistersData *data, EthernetClient *client)
{

    bool sent = false;
    uint8_t i, size;
    if(client->connected()){
    client->print("GET http://");
    client->print(serverIp);
    client->print("/smartifsc/php/pm210.php");

    //enviando id_device
    client->print("&id_device=");
    client->print(data->id_device, HEX);
    client->print(",");
    client->print(data->id_device, HEX);

    //enviando op_mode
    client->print("&op_mode=");
    client->print(data->op_mode, HEX);
    client->print(",") ;
    client->print(data->op_mode, HEX);

    //enviando vendor_op_mode
    client->print("&vendor_op_mode=");
    client->print(data->vendor_op_mode, HEX);
    client->print(",") ;
    client->print(data->vendor_op_mode, HEX);

    //enviando tot_accurrent
    client->print("&tot_accurrent=");
    client->print(data->tot_accurrent, HEX);
    client->print(",") ;
    client->print(data->tot_accurrent, HEX);

    //enviando phase_current
    client->print("&phase_current=");
    size = sizeof(data->phase_current) / sizeof(data->phase_current[0]);
    for( i=0 ; i < size ; i++){
      client->print(data->phase_current[i], HEX);
      client->print(",");
      }
    client->print(data->phase_current[i]);

    //enviando sf_accurrent
    client->print("&sf_accurrent=");
    client->print(data->sf_accurrent, HEX);
    client->print(",");
    client->print(data->sf_accurrent, HEX);

    //enviando voltage_ph_ph
    client->print("&voltage_ph_ph=");
    size = sizeof(data->voltage_ph_ph) / sizeof(data->voltage_ph_ph[0]);
        for( i=0 ; i < size ; i++){
            client->print(data->voltage_ph_ph[i], HEX);
            client->print(",");
        }
    client->print(data->voltage_ph_ph[i]);

    //enviando voltage_ph_nt
    client->print("&voltage_ph_nt=");
    size = sizeof(data->voltage_ph_nt) / sizeof(data->voltage_ph_nt[0]);
        for( i=0 ; i < size ; i++){
            client->print(data->voltage_ph_nt[i], HEX);
            client->print(",");
        }
    client->print(data->voltage_ph_nt[i]);

    //enviando sf_voltage
    client->print("&sf_voltage=");
    client->print(data->sf_voltage, HEX);
    client->print(",");
    client->print(data->sf_voltage, HEX);

    //enviando ac_infeed_sf
    client->print("&ac_infeed_sf=");
    size = sizeof(data->ac_infeed_sf) / sizeof(data->ac_infeed_sf[0]);
        for( i=0 ; i < size ; i++){
            client->print(data->ac_infeed_sf[i], HEX);
            client->print(",");
        }
    client->print(data->ac_infeed_sf[i]);

    //enviando ac_freq
    client->print("&ac_freq=");
    client->print(data->ac_freq, HEX);
    client->print(",");
    client->print(data->ac_freq, HEX);

    //enviando sf_ac_freq
    client->print("&sf_ac_freq=");
    client->print(data->sf_ac_freq, HEX);
    client->print(",");
    client->print(data->sf_ac_freq, HEX);

    //enviando powergen_sf
    client->print("&powergen_sf=");
    size = sizeof(data->powergen_sf) / sizeof(data->powergen_sf[0]);
        for( i=0 ; i < size ; i++){
            client->print(data->powergen_sf[i], HEX);
            client->print(",");
        }
    client->print(data->powergen_sf[i]);

    //enviando tot_fedin_energy
    client->print("&tot_fedin_energy=");
    size = sizeof(data->tot_fedin_energy) / sizeof(data->tot_fedin_energy[0]);
        for( i=0 ; i < size ; i++){
            client->print(data->tot_fedin_energy[i], HEX);
            client->print(",");
        }
    client->print(data->tot_fedin_energy[i]);

    //enviando sf_tot_fedin
    client->print("&sf_tot_fedin=");
    client->print(data->sf_tot_fedin, HEX);
    client->print(",");
    client->print(data->sf_tot_fedin, HEX);

    //enviando dc_current
    client->print("&dc_current=");
    client->print(data->dc_current, HEX);
    client->print(",");
    client->print(data->dc_current, HEX);

    //enviando sf_dc_current
    client->print("&sf_dc_current=");
    client->print(data->sf_dc_current, HEX);
    client->print(",");
    client->print(data->sf_dc_current, HEX);

    //enviando dc_voltage
    client->print("&dc_voltage=");
    client->print(data->dc_voltage, HEX);
    client->print(",");
    client->print(data->dc_voltage, HEX);

    //enviando sf_dc_voltage
    client->print("&sf_dc_voltage=");
    client->print(data->sf_dc_voltage, HEX);
    client->print(",");
    client->print(data->sf_dc_voltage, HEX);

    //enviando dc_pwgen_stat_sf
    client->print("&dc_pwgen_stat_sf=");
    size = sizeof(data->dc_pwgen_stat_sf) / sizeof(data->dc_pwgen_stat_sf[0]);
        for( i=0 ; i < size ; i++){
            client->print(data->dc_pwgen_stat_sf[i], HEX);
            client->print(",");
        }
    client->print(data->dc_pwgen_stat_sf[i]);

    // info
    client->println("HTTP/1.1");
    client->println( "Host: smartifsc" );
    client->println( "Content-Type: application/x-www-form-urlencoded" );
    client->println( "Connection: close" );
    client->println();
    client->println();
    client->stop();
    client->flush();
    sent = true;
    }
    else{//cliente não conectado
      #ifdef DEBUG
      debug.write("client not connected. \n");
      debug.flush();
      #endif
            client->stop();
            client->flush();
            sent = false;

      }
      return sent;
}*/

