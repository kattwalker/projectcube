#include <stdint.h>

#include <SoftwareSerial.h>
#include <AltSoftSerial.h>

SoftwareSerial Serial4(12, 13);

void send_message();
static const float SCALE_VALUE = 127; 
void receieve_message();
int receive_neighbour_123(HardwareSerial& neighbour_serial, float* message, int read_signal);
int receive_neighbour_4(SoftwareSerial& neighbour_serial, float* message, int read_signal);
void update_message();
unsigned long now = 0;
unsigned long prev = 0;
const unsigned long math_trigger = 50000;
int amount_of_cell_info = 11;
int neural_network_parameter = 6;

float cell_state[11] = {1,0,0,0,0,0,0,0,0,0,0};

// n-b better to change to n,s,e,w
float ReadFromRightMessage[11] = {0,0,0,0,0,0,0,0,0,0,0};
float ReadFromLeftMessage[11] = {0,0,0,0,0,0,0,0,0,0,0};
float ReadFromTopMessage[11] = {0,0,0,0,0,0,0,0,0,0,0};
float ReadFromBottomMessage[11] = {0,0,0,0,0,0,0,0,0,0,0};

int message_sent = 0;

static const int red_light_pin= 7;
static const int green_light_pin = 6;
static const int blue_light_pin = 5;

int message_process_completed = 0;
int bottom_read = 0;
int right_read = 0;
int left_read = 0;
int top_read =0;
int ready_to_update = 0;
float loop_count = 0;

          

const float dmodel_kernel_2[6][11] = {{ 0.00702192, -0.0302443,   0.01429643, -0.22756426,  1.2955177,
    0.00752916, -0.29012364,  0.08942816, -0.01615511,  0.06804911,
   -0.2836687},
  { 0.04007224,  1.3502132,   0.11379168, -0.31065598, -0.04229368,
    0.00265359, -0.2836391,  -0.25732392, -0.04549034, -0.24385932,
    0.07402212},
  {-0.35909122, -0.22622462,  0.28077918,  0.10340051, -0.02087522,
    0.5709186,   0.10269472,  0.01850549, -0.93992895,  0.00838773,
    0.0336612 },
  {-0.01846585,  0.01981815,  0.02233427, -0.14033619, -0.03081362,
   -0.00536005, -0.16720532,  0.42700145, -0.0144696,   0.38336337,
   -0.3424963 },
  {-0.03769223, -0.54387647, -0.06379467,  0.09657511,  0.01296606,
   -0.01034394,  0.05741742,  0.10823763,  0.59571296,  0.08341924,
   -0.34774488},
  { 0.12619065, -0.0155779,  -0.01518512, -0.2595486,  -0.01915251,
   -0.00971039, -0.21480866,  0.08380179, -0.01003404,  0.1077911,
    0.30996972}};
   
 const float dmodel_kernel_1[6][6]  = {{ 0.8979443,  -1.0128694,  -0.3487833,  -0.7109052,  -0.6384581,  -0.01903137},
 { 0.10997926,  0.35088706,  0.5691418,   0.8884787,  -0.2820465,   0.46501493},
 {-1.1904217,  -0.14769061, -0.20392242,  1.1553392,  -0.4069939,   0.95524937},
 { 0.41436547, -1.1071093,  -0.9457747,  -0.3253983,  -0.91521883, -0.09872462},
 { 0.01729763, -0.10395884,  0.6100858,  -0.25196585,  0.8350334,   0.9910932 },
 {-1.3845084,  -0.56525826, -2.216473 ,   0.69323087, -0.45990723,  0.7144988 }};



const float perceive_kernel[3][3][11][6] = {{
  
  {{-0., -0.,  0., -0.,  0., -0.,},
 {-0.,  0.,  0., -0., -0., -0.,},
 {-0., -0.,  0., -0.,  0., -0.,},
 { 0.,  0., -0., -0.,  0.,  0.,},
 { 0.,  0.,  0.,  0.,  0., -0.,},
 {-0., -0., -0., -0.,  0., -0.,},
 { 0.,  0., -0., -0.,  0., -0.,},
 {-0.,  0.,  0., -0., -0.,  0.,},
 { 0., -0.,  0., -0.,  0.,  0.,},
 {-0.,  0.,  0., -0., -0.,  0.,},
 { 0.,  0.,  0.,  0., -0.,  0.,}},{{-1.74338490e-01, -9.75546837e-02, -2.16860488e-01,  6.50515780e-02,
  -3.94157926e-03,  2.87146658e-01,},
 {-1.47932053e-01,  2.68268198e-01,  4.70457003e-02, -5.47603846e-01,
  -2.59130776e-01, -1.21725656e-01,},
 {-2.10001126e-01,  4.01430130e-01,  1.76024958e-01, -1.09087968e+00,
   1.34017793e-02, -5.61211467e-01,},
 { 2.17917204e-01,  2.70741922e-03, -1.69487875e-02,  1.59655735e-01,
  -2.36755416e-01, -1.62822932e-01,},
 {-6.36959672e-01,  2.30768487e-01, -9.53314379e-02,  9.47933733e-01,
  -2.06799850e-01, -7.94137716e-01,},
 {-9.82381627e-02, -1.99581627e-02,  1.84234440e-01, -9.93006825e-01,
   2.29341209e-01, -5.04667282e-01,},
 { 2.30479747e-01, -5.17933108e-02,  3.54354158e-02,  1.48060426e-01,
  -7.10761845e-02, -1.33053407e-01,},
 { 6.48269802e-02,  2.39568651e-02,  6.78714961e-02, -1.90258771e-01,
  -8.43397141e-01,  5.78646779e-01,},
 { 5.61528177e-05, -6.96922019e-02, -2.23313212e-01, -3.21502298e-01,
   1.76339373e-01,  1.50961608e-01,},
 { 5.66018596e-02,  8.78856052e-03,  1.07076496e-01, -1.84065312e-01,
  -7.87765622e-01,  5.59366167e-01,},
 { 8.32890496e-02, -2.27462798e-01,  2.36944973e-01,  2.89882153e-01,
   2.92054474e-01,  8.15689340e-02,}},
{{-0., -0.,  0.,  0.,  0.,  0.},
 {-0.,  0., -0., -0., -0.,  0.},
 {-0.,  0.,  0.,  0., -0.,  0.},
 { 0., -0., -0.,  0., -0., -0.},
 { 0.,  0., -0.,  0., -0., -0.},
 {-0.,  0.,  0.,  0., -0.,  0.},
 {-0., -0., -0.,  0., -0., -0.},
 {-0.,  0.,  0., -0., -0.,  0.},
 {-0.,  0., -0., -0.,  0., -0.},
 {-0.,  0.,  0., -0., -0. , 0.},
 {-0., -0.,  0., -0.,  0., -0.}}},
 
 
 
{{{-1.5060730e-01, -2.3086917e-01,  3.6193731e-01,  2.5952703e-01,
  -4.4510812e-02, -7.1525900e-03,},
 {-5.2980192e-02, -1.5467215e-01,  3.3792943e-02, -6.8422137e-03,
  -1.9725449e-01,  1.1729214e-01,},
 {-1.8570249e-01, -9.9504143e-01, -1.1124284e-02,  1.2254280e+00,
  -1.0578896e-01,  8.5897259e-02,},
 {-3.0331151e-04,  1.9205943e-01, -2.3919846e-01,  1.5063910e-01,
   9.6388325e-02, -4.9617056e-02,},
 { 5.4443702e-02,  3.0691385e-01, -1.7103530e-01,  6.5732437e-01,
  -2.8474426e-01, -1.9448901e+00,},
 { 8.8649698e-02, -4.7616041e-01, -2.2787662e-01 ,-1.5639491e-01,
   1.3016599e-01,  9.4663791e-02,},
 { 3.8151236e-03,  2.3684661e-01, -2.5267884e-01,  8.0701828e-02,
   2.6164737e-01,  2.0012854e-01,},
 {-4.1533792e-01, -7.5367022e-01,  8.2262802e-01,  1.1711613e-01,
  -4.5469818e-01, -5.1114571e-01,},
 {-8.2901943e-01, -1.9311629e-01, -7.8628451e-01, -6.4375407e-01,
   2.7389053e-01,  2.1266565e-01,},
 {-4.3672666e-01, -8.8538426e-01,  8.5053390e-01,  9.4777159e-02,
  -3.8186663e-01, -4.6011883e-01,},
 {-1.1028645e-01,  1.4541051e-02, -5.2888770e-02,  4.7720200e-03,
   1.0231016e-01,  1.7557593e-01}},
  {{-0.01299959,  0.0939208 , -0.01471367, -0.02065544,  0.0439869,  -0.2738984},
 { 0.75604117, -0.35647205, -0.15954386,  0.30041602,  0.6509734,   0.16332568},
 { 0.76940423, -0.46136928,  0.09334536,  0.39397818, -0.13998465,  0.41869912},
 {-0.10782582,  0.3128168,   0.2028541,   0.05735997,  0.3681136,   0.18853803},
 {-0.48044646, -0.4030752,   0.44780174, -0.84105384, -0.54071563,  0.47484443},
 { 0.61497086, -0.51077855,  0.41065943, -0.10412422, -0.4748708,   0.30579877},
 {-0.12109882,  0.3717019,   0.19292723,  0.09099651,  0.27114275,  0.142013  },
 { 0.17639327, -0.6464453,  -0.7549358,   0.1327702,  -0.97422653, -0.66118264},
 { 0.1764005 ,  0.20466262,  0.23929243,  0.08209541, -0.32800308, -0.01794848},
  { 0.08369324, -0.641041,   -0.7630897,   0.09460241, -1.0234668,  -0.63704646},
 {-0.22715184,  0.29507372,  0.04762322, -0.05352475, -0.30565813, -0.08571813}}, {{ 3.52414489e-01, -1.90665796e-01, -2.58581936e-01, -6.74946234e-02,
  -6.35313541e-02,  3.80383879e-01},
 {-5.28132558e-01,  2.95168847e-01,  5.58621585e-02, -9.36462939e-01,
  -3.87060344e-01, -1.23837478e-01},
 {-8.77523124e-01,  5.73520720e-01, -1.88941136e-01, -1.68441570e+00,
   1.31642269e-02, -7.08711207e-01},
 { 1.26625106e-01,  1.08845621e-01, -1.06568085e-02, -6.06294610e-02,
  -2.72254944e-01,  2.25180127e-02},
 { 5.04528522e-01,  5.08697510e-01, -2.46941581e-01,  9.03739855e-02,
  -5.46085536e-01, -9.85333025e-02},
 {-8.97056222e-01,  1.88716218e-01, -2.41066992e-01, -9.54905093e-01,
   2.95617521e-01, -5.24141252e-01},
 { 1.80600166e-01, -3.87443788e-02, -5.24063641e-03, -6.08058125e-02,
  -1.50354942e-02,  5.04491515e-02},
 {-9.50101733e-01, -1.17111057e-01,  5.58627695e-02,  3.79631788e-01,
  -1.05285370e+00, -5.72361313e-02},
 {-2.85819560e-01,  4.71599810e-02,  1.49906799e-01, -1.33521797e-03,
   5.15942788e-03,  4.72679455e-03},
 {-8.90944719e-01, -1.80186674e-01,  6.21654242e-02,  3.82451236e-01,
  -9.70917761e-01, -3.82724628e-02},
 { 2.32153699e-01, -3.30378681e-01,  4.51830626e-02,  3.79307754e-02,
   3.84642512e-01,  8.06693137e-02}}},
{{{ 0., -0., -0.,  0.,  0., -0.,},
 {-0., -0.,  0.,  0.,  0., -0.,},
 { 0., -0., -0.,  0., -0., -0.,},
 { 0.,  0., -0.,  0., -0.,  0.,},
 { 0.,  0.,  0., -0., -0.,  0.,},
 {-0.,  0., -0.,  0., -0., -0.,},
 { 0.,  0., -0., -0., -0.,  0.,},
 {-0.,  0.,  0., -0., -0.,  0.,},
 {-0.,  0.,  0., -0., -0.,  0.,},
 {-0., -0.,  0., -0.,  0., -0.,},
 { 0.,  0., -0., -0., -0., -0.,}},
 {{-0.17127691,  0.05881327, -0.30106828,  0.12195411, -0.06637431,  0.25354317},
 {-0.2414661,   0.10997754,  0.13167112, -0.74811304, -0.26369947, -0.01895403},
 {-0.17345706,  0.21413572,  0.11061712, -1.6204771,   0.02776619, -0.21619998},
 {-0.09600174, -0.25203887,  0.17991124,  0.26757672,  0.00333811, -0.16032565},
 {-0.28010368,  0.12861578,  0.13216616,  0.12732427,  0.39335608,  0.16906537},
 {-0.11471849, -0.28289276,  0.00239785, -0.5153118,   0.21658425, -0.2405685 },
 {-0.08725742, -0.28418544,  0.07464097,  0.28346568,  0.04465878, -0.10483871},
 { 0.56728345,  0.80164367,  0.18127784,  0.20171855, -1.2359532,   0.30696872},
 { 0.04671061, -0.33556607, -0.5188363 , -0.30994782,  0.21713826,  0.03661957},
 { 0.58152765,  0.7959953,   0.08500461,  0.24442351, -1.2511172,   0.29357058},
 { 0.14155227, -0.10289618, -0.18802792,  0.3403368,   0.0023156,   0.13541469}},{{ 0.,  0., -0.,  0.,  0.,  0.},
 {-0.,  0.,  0., -0., -0.,  0.,},
 {-0.,  0.,  0., -0., -0.,  0.,},
 {-0.,  0., -0.,  0.,  0., -0.,},
{ 0., -0., -0.,  0., -0., -0.,},
 {-0.,  0.,  0., -0., -0.,  0.,},
 {-0.,  0., -0., -0.,  0., -0.,},
 {-0.,  0.,  0., -0., -0.,  0.,},
 { 0., -0., -0.,  0., -0.,  0.,},
 {-0.,  0.,  0., -0., -0.,  0.,},
{-0., -0., -0., -0.,  0., -0.,}}}};


float dmodel_bias_1[6] = {0.012902863,0.20334744,0.047111467,-0.04879921,0.11107109,-0.1389306};
float dmodel_bias_2[11] = {-0.008380199,0.0010055156,-0.0015280047,0.057577252,0.002598451,-0.0008966971,0.056395955,-0.035218723,0.0014696647,-0.031519607,0.01778244};
float perceive_bias[6] = {-0.06075142,0.0652175,0.11544124,-0.08911467,0.11749506,-0.11390978};

void setup() {
        Serial.begin(9600); // to pc
        Serial1.begin(9600); //Left neighbour
        Serial2.begin(9600);// right neighbour
        Serial3.begin(9600); // top neighbour

        Serial4.begin(9600); // bottom neighbour

        pinMode(7, OUTPUT);
        pinMode(6, OUTPUT);
        pinMode(5, OUTPUT);

}

void loop() { 
         
  send_message();
  delay(500);
  receieve_message();      
  update_message();
  }


void send_message()
{
  

  ////////////////// send message!!!! //////////////////////////////////
  if(message_process_completed == 0){ //only send one message at a time - wait for the neural network to update 
    //before sending another one
    message_process_completed = 1; //reset trigger

    // send header... at the moment its randomly 25 but we can change later if required
    uint8_t header = 25;
    // send header to each of the neighbours
    Serial1.write(header);
    Serial2.write(header);
    Serial3.write(header);
    Serial4.write(header);

    for (int k = 0; k < amount_of_cell_info; k++){
    // send the current cell state to each of its neighbours
      uint8_t x = cell_state[k]*SCALE_VALUE+127; // convert into sendable format (i.e. cell_state of -10 --> 0 cells state of +10 --> 255)
      // send to each neighbour
      Serial1.write(x);
      Serial2.write(x);
      Serial3.write(x);
      Serial4.write(x);}

    // set message sent trigger - actually this might be obsolete now ...
    message_sent = 1;}
}

void receieve_message()
{
  bottom_read = receive_neighbour_123(Serial2,  ReadFromBottomMessage, bottom_read);
  top_read = receive_neighbour_123(Serial1,  ReadFromTopMessage, top_read);
  right_read = receive_neighbour_123(Serial3,  ReadFromRightMessage, right_read);
  left_read = receive_neighbour_4(Serial4, ReadFromLeftMessage, left_read);
}

int receive_neighbour_123(HardwareSerial& neighbour_serial, float* message, int read_signal)
{
  ///////////////////////////////Read message !!!!!////////////////////////////////////
  // We are constantly checking for send data from each of the neighbours - also only read once per update  
  if(neighbour_serial.available()>0 && read_signal ==0){
    
    int value = neighbour_serial.read(); // check for the header value - only start recording when you read this value
    
    if(value == 25){
      // if the header value is read, read the next 11 numbers, which is the neighbouring cell state.N.B. this needs to be converted back into 
      // the neural network weight format.
      for(int i = 0; i < amount_of_cell_info; i++){
        if(neighbour_serial.available()>0){
          uint8_t temp = (neighbour_serial.read()); 
          message[i] = ((int16_t)temp-127)/SCALE_VALUE;
       
        }
        else
        {
          Serial.println("no data");
        }
      }
      // set bottom read to one, this neighbour has now been read
      return 1;       
    }
  }
  return read_signal;
}

int receive_neighbour_4(SoftwareSerial& neighbour_serial, float* message, int read_signal)
{
  ///////////////////////////////Read message !!!!!////////////////////////////////////
  // We are constantly checking for send data from each of the neighbours - also only read once per update  
  if(neighbour_serial.available()>0 && read_signal ==0){
    
    int value = neighbour_serial.read(); // check for the header value - only start recording when you read this value
    if(value == 25){
      // if the header value is read, read the next 11 numbers, which is the neighbouring cell state.N.B. this needs to be converted back into 
      // the neural network weight format.
      for(int i = 0; i < amount_of_cell_info; i++){
        if(neighbour_serial.available()>0){
          uint8_t temp = (neighbour_serial.read()); 
          message[i] = ((int16_t)temp-127)/SCALE_VALUE;
         
        }
      }
      // set bottom read to one, this neighbour has now been read
      return 1;       
    }
  }
  return 0;
}

void update_message()
{
////////////////////////// Update message ///////////////////////////////////////////
  // we only attempt at update every 100th loop - this should give the algorithm enough time to
  //read the sent message in the correct order 
  now = millis();
  unsigned long check = now - prev; 
  if((check > math_trigger) && (bottom_read == 1 and top_read == 1)){
    
    prev = now;
    if(bottom_read == 1){Serial.println("up");
      for(int j = 0 ; j < amount_of_cell_info ; j++ ) {Serial.println(ReadFromBottomMessage[j]);}}
    if(top_read == 1){Serial.println("down");
      for(int j = 0 ; j < amount_of_cell_info ; j++ ) {Serial.println(ReadFromTopMessage[j]);}}
    if(left_read == 1){Serial.println("left");
      for(int j = 0 ; j < amount_of_cell_info ; j++ ) {Serial.println(ReadFromLeftMessage[j]);}}
    if(right_read == 1){Serial.println("right");
      for(int j = 0 ; j < amount_of_cell_info ; j++ ) {Serial.println(ReadFromRightMessage[j]);}}


    // update the cell state part... 
    float sumA1[neural_network_parameter] = {0,0,0,0,0,0};
    float sumA2[neural_network_parameter] = {0,0,0,0,0,0};
    float sumA3[neural_network_parameter] = {0,0,0,0,0,0};

    for(int j = 0 ; j < neural_network_parameter ; j++ ) {
      for(int i = 0 ; i < amount_of_cell_info ; i++ ) {
          sumA1[j] += (perceive_kernel[1][1][i][j] * cell_state[i]); }}


    for(int j = 0 ; j < neural_network_parameter ; j++ ) {
      for(int i = 0 ; i < amount_of_cell_info ; i++ ) {
          sumA1[j] += (perceive_kernel[0][1][i][j] * ReadFromTopMessage[i]) ; }}

    for(int j = 0 ; j < neural_network_parameter ; j++ ) {
      for(int i = 0 ; i < amount_of_cell_info ; i++ ) {
          sumA1[j] += (perceive_kernel[2][1][i][j] *  ReadFromBottomMessage[i]) ; }}

     

    for(int j = 0 ; j < neural_network_parameter ; j++ ) {
      for(int i = 0 ; i < amount_of_cell_info ; i++ ) {
          sumA1[j] += (perceive_kernel[2][1][i][j] * ReadFromRightMessage[i]) ; }}

    for(int j = 0 ; j < neural_network_parameter ; j++ ) {
      for(int i = 0 ; i < amount_of_cell_info ; i++ ) {
          sumA1[j] += (perceive_kernel[1][0][i][j] *  ReadFromLeftMessage[i]) ; }}

    

    for(int j = 0 ; j < neural_network_parameter ; j++ ) {sumA1[j] += perceive_bias[j];}
    

    
    
    for(int j = 0 ; j < neural_network_parameter ; j++ ) {if(sumA1[j]<0){sumA1[j]=0;}}
    
    


    float sumB[neural_network_parameter] = {0,0,0,0,0,0};
    for(int j = 0 ; j < neural_network_parameter ; j++ ) {        
      for(int i = 0 ; i < neural_network_parameter; i++ ) {
        sumB[j] += sumA1[i] *dmodel_kernel_1[i][j];
      }
    }


    for(int j = 0 ; j < neural_network_parameter; j++ ) {
      sumB[j] += dmodel_bias_1[j];
      if(sumB[j]<0){
        sumB[j] = 0;
      }
    }
    
    

  float sumC[amount_of_cell_info] = {0,0,0,0,0,0,0,0,0,0,0};

  for(int j = 0 ; j< amount_of_cell_info ; j++ ) {        
    for(int i = 0 ; i < neural_network_parameter; i++ ) {
      sumC[j] =+ (sumB[i] *dmodel_kernel_2[i][j]);
    }
  }

  
  Serial.print("ITS THIS BIT THATS NOT WORKING ANDRES...");
  for(int j = 0 ; j < amount_of_cell_info ; j++ ) {Serial.println(sumC[j]);}
  delay(1000);

  for(int j = 0 ; j < amount_of_cell_info ; j++ ) {sumC[j] += dmodel_bias_2[j];}
  
 
  //for(int j = 0 ; j < amount_of_cell_info ; j++ ) {cell_state[j] += sumC[j];}


  // now we find out which number the cell thinks it is...
  // looks for the position of the maximum number of the last 10 digits..
  float max_num = -10;
  int max_position = 0;
  for(int i=1; i<amount_of_cell_info; i++){
    // we can also print it for debugging...
    Serial.print("me..");
    Serial.println(cell_state[i]);
    if(cell_state[i]>max_num){
      max_num =cell_state[i]; 
      max_position=i;
    }
  }

  // set the light colour accordingly...

  if(max_position==1){analogWrite(red_light_pin, 255); analogWrite(green_light_pin,0); analogWrite(blue_light_pin, 0);}
  if(max_position==2){analogWrite(red_light_pin, 0); analogWrite(green_light_pin,255); analogWrite(blue_light_pin, 0);}
  if(max_position==3){analogWrite(red_light_pin, 0); analogWrite(green_light_pin,0); analogWrite(blue_light_pin, 255);}
  if(max_position==4){analogWrite(red_light_pin, 255); analogWrite(green_light_pin,255); analogWrite(blue_light_pin, 125);}
  if(max_position==5){analogWrite(red_light_pin, 0); analogWrite(green_light_pin,255); analogWrite(blue_light_pin, 255);}
  if(max_position==6){analogWrite(red_light_pin, 255); analogWrite(green_light_pin,0); analogWrite(blue_light_pin, 255);}
  if(max_position==7){analogWrite(red_light_pin, 255); analogWrite(green_light_pin,255); analogWrite(blue_light_pin, 0);}
  if(max_position==8){analogWrite(red_light_pin, 255); analogWrite(green_light_pin,255); analogWrite(blue_light_pin, 255);}
  if(max_position==9){analogWrite(red_light_pin, 255); analogWrite(green_light_pin,125); analogWrite(blue_light_pin, 255);}
  if(max_position==10){analogWrite(red_light_pin, 125); analogWrite(green_light_pin,255); analogWrite(blue_light_pin, 255);}


  Serial.println("I think I am...");
  Serial.println((max_position-1));

  // the update process is now completed - update each of the necessary parameters / reset the triggers...
  message_process_completed = 0;
  bottom_read = 0;
  top_read = 0;
  left_read = 0;
  right_read = 0;
  ready_to_update = 0;
  for(int i=1; i<amount_of_cell_info; i++){
    ReadFromRightMessage[i] = 0;
    ReadFromTopMessage[i] = 0;
    ReadFromBottomMessage[i] = 0;
    ReadFromLeftMessage[i] = 0;
    }
  
  }

}
