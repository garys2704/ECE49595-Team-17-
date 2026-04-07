/*
 * ili9341_GFX.c
 *
 *  Created on: Mar 31, 2026
 *      Author: Matthew Mozingo
 */

//Step 1
StatusPOR = ReadRegister(0x00) & 0x0002;
if (StatusPOR==0){goto Step 4.3;} //then go to Step 4.3.
else { //then do Steps 2–3.}
}


//Step 2
while(ReadRegister(0x3D) & 1){}
Wait(10); //do not continue until FSTAT.DNR == 0


//Step 3
HibCFG=ReadRegister(0xBA) ;     //Store original HibCFG value
WriteRegister (0x60 , 0x90)   ; // Exit Hibernate Mode step 1
WriteRegister (0xBA , 0x0)    ; // Exit Hibernate Mode step 2
WriteRegister (0x60 , 0x0)    ; // Exit Hibernate Mode step 3



//Step 4
Status = ReadRegister(0x00);             //Read Status
WriteAndVerifyRegister (0x00, Status AND 0xFFFD) ;  //Write and Verify Status with POR bit cleared


//Step 4.2
StatusPOR = ReadRegister(0x00) & 0x0002; //Read POR bit in Status register
If StatusPOR = 0, then go to Step 4.3.
If StatusPOR = 1, then go to Step 1.

//Step 4.3
RepCap = ReadRegister(0x05) ;    //Read RepCap
RepSOC = ReadRegister(0x06)  ;    //Read RepSOC

//Step 4.4
TTE = ReadRegister(0x11) ;       //Read TTE

//Step 4.5
Saved_RCOMP0 = ReadRegister(0x38)   ;  //Read RCOMP0
Saved_TempCo = ReadRegister(0x39)   ;  //Read TempCo
Saved_FullCapRep = ReadRegister(0x10)  ;  //Read FullCapRep
Saved_Cycles = ReadRegister(0x17)   ;  //Read Cycles
Saved_FullCapNom = ReadRegister(0x23) ;  //Read FullCapNom


//Step 4.6
WriteAndVerifyRegister(0x38, Saved_RCOMP0)   ;   //WriteAndVerify RCOMP0
WriteAndVerifyRegister(0x39, Saved_TempCo)   ;   //WriteAndVerify TempCo
WriteAndVerifyRegister(0x23, Saved_FullCapNom) ; //WriteAndVerify FullCapNom

//Step 4.7 Wait 350 ms


//Step 4.8
FullCapNom= ReadRegister(0x23)
         ;
 //Read FullCapNom
MixCap=(ReadRegister(0x0D)*FullCapNom)/25600  ;
WriteAndVerifyRegister(0x0F, MixCap)      ; //WriteAndVerify MixCap
WriteAndVerifyRegister(0x10, Saved_FullCapRep)   ; //WriteAndVerify FullCapRep
//Write dQacc to 200% of Capacity and dPacc to 200%
dQacc = (Saved_FullCapNom/ 16)     ;
WriteAndVerifyRegister (0x46, 0x0C80)  ;    //Write and Verify dPacc
WriteAndVerifyRegister (0x45, dQacc)  ;    //Write and Verify dQacc


//Step 4.10 Wait 350 ms

//Step 4.11
WriteAndVerifyRegister(0x17, Saved_Cycles) ;    //WriteAndVerify Cycles



m
int WriteRegister (u8 reg, u16 value)
 {
     int ret = i2c_smbus_write_word_data(client, reg, value);

     if (ret < 0)
         dev_err(&client->dev, "%s: err %d\n", __func__, ret);
     return ret;
 }

int ReadRegister (u8 reg)
 {
     int ret = i2c_smbus_read_word_data(client, reg);

     if (ret < 0)
         dev_err(&client->dev, "%s: err %d\n", __func__, ret);
     return ret;
 }

void WriteAndVerifyRegister (char RegisterAddress, int RegisterValueToWrite){
  int Attempt=0;
  do {
    WriteRegister (RegisterAddress, RegisterValueToWrite);
    Wait(1); //1ms
    RegisterValueRead = ReadRegister (RegisterAddress) ;
 }
while (RegisterValueToWrite != RegisterValueRead && attempt++<3);
}

