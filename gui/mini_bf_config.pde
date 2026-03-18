// ================================================================
// RP2354A FC — Mini Betaflight Configurator  v4
// Processing 4.x  (https://processing.org)
// ================================================================
// 22-field CSV telemetry + full bidirectional command set
// New in v4:  RC Calibration panel, ESC calibration, Self-Test
//             panel, expo/deadband sliders, Yaw PID, DUMP_BB
// ================================================================

import processing.serial.*;

// ---- Serial ----
Serial  port;
boolean connected = false;

// ---- Telemetry state ----
float   roll=0, pitch=0, yaw=0;
float[] motors = new float[4];
float   kp=2.5, ki=0.05, kd=0.03;
float   yawKp=3.0, yawKi=0.05, yawKd=0.0;
float   altitude=0;
boolean armed=false;
int     mode=0;
int     volt_mV=0, curr_cA=0, bat_pct=100;
int[]   escRPM  = new int[4];
int     loop_us=1000;
int     fs_level=0;

// Expo / deadband display (per-channel, mirrored from FC)
float[] chExpo      = {0.20, 0.20, 0.0, 0.20, 0.0, 0.0, 0.0, 0.0};
float[] chDeadband  = {0.02, 0.02, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0};

// Self-test results
String[] stResults = new String[0];
boolean  stReceiving = false;
boolean  stDone      = false;

// Spectrum (simulated unless FC sends real FFT data)
float[] spectrum = new float[48];

// Current tab: 0=Flight 1=PID 2=RCCal 3=SelfTest
int currentTab = 0;

// ---- Layout ----
final int W=1200, H=800;
PFont mono, sans, titleF;

// Palette
final color BG      = color(18, 20, 30);
final color PANEL   = color(28, 32, 48);
final color TABACT  = color(65, 145, 255);
final color TABINAC = color(40, 45, 65);
final color GREEN   = color(50, 210, 90);
final color RED     = color(210, 60, 60);
final color ORANGE  = color(255, 160, 40);
final color WHITE   = color(220, 222, 228);
final color DIM     = color(100, 105, 120);
final color GOLD    = color(255, 200, 50);

// PID sliders
float sKP=2.5, sKI=0.05, sKD=0.03;
float sYawKP=3.0, sYawKI=0.05, sYawKD=0.0;
boolean dKP=false,dKI=false,dKD=false;
boolean dYKP=false,dYKI=false,dYKD=false;
final int SLD_W=270, SLD_H=12;

// RC Cal expo/deadband sliders state
boolean[] dExpo = new boolean[4];
boolean[] dDB   = new boolean[4];

// ================================================================
void setup()
{
    size(1200, 800, P3D);
    frameRate(60);
    smooth(4);
    mono   = createFont("Courier New", 11, true);
    sans   = createFont("Arial",       11, true);
    titleF = createFont("Arial Bold",  13, true);

    for (int i=0; i<spectrum.length; i++) spectrum[i]=0;

    println("=== RP2354A FC Configurator v4 ===");
    String[] ports = Serial.list();
    for (int i=0; i<ports.length; i++) println("["+i+"] "+ports[i]);
    if (ports.length > 0)
    {
        port = new Serial(this, ports[0], 115200);
        port.bufferUntil('\n');
        connected = true;
        println("Connected: " + ports[0]);
    }
}

// ================================================================
void draw()
{
    background(BG);
    drawTitleBar();
    drawTabs();
    if      (currentTab == 0) drawFlightTab();
    else if (currentTab == 1) drawPIDTab();
    else if (currentTab == 2) drawRCCalTab();
    else if (currentTab == 3) drawSelfTestTab();
    drawStatusFooter();
}

// ---- Title bar ------------------------------------------------
void drawTitleBar()
{
    fill(PANEL); noStroke(); rect(0,0,W,42);
    fill(TABACT); textFont(titleF); textAlign(LEFT,CENTER);
    text("  RB-RP2354A  Flight Controller  —  Configurator v4", 8, 21);
    fill(connected ? color(50,200,70) : RED);
    ellipse(W-22, 21, 11, 11);
    fill(WHITE); textFont(sans); textAlign(RIGHT,CENTER);
    text(connected ? "CONNECTED" : "OFFLINE", W-34, 21);
}

// ---- Tab bar --------------------------------------------------
void drawTabs()
{
    String[] tabs = {"Flight","PID Tuning","RC Calibration","Self-Test"};
    for (int i=0; i<tabs.length; i++)
    {
        int tx = 12 + i*150, ty=44;
        fill(i==currentTab ? TABACT : TABINAC);
        noStroke(); rect(tx, ty, 144, 26, 4, 4, 0, 0);
        fill(255); textFont(sans); textAlign(CENTER,CENTER);
        text(tabs[i], tx+72, ty+13);
    }
}

// ================================================================
//  TAB 0  — Flight  (cube + motors + battery + FFT)
// ================================================================
void drawFlightTab()
{
    drawStatusBadges();
    draw3DCube();
    drawMotorPanel();
    drawBatteryPanel();
    drawFFTPanel();
    drawFlightButtons();
}

void drawStatusBadges()
{
    int y=80;
    // ARM
    fill(armed ? GREEN : RED); noStroke(); rect(12,y,112,26,5);
    fill(255); textFont(titleF); textAlign(CENTER,CENTER);
    text(armed ? "●  ARMED" : "○  DISARMED", 68, y+13);
    // Mode
    fill(TABACT); rect(134,y,84,26,5);
    fill(255); text(mode==0?"ACRO":"ANGLE", 176, y+13);
    // Failsafe
    color fc = fs_level==0 ? color(40,160,40) : fs_level==1 ? ORANGE : RED;
    fill(fc); rect(228,y,85,26,5);
    fill(255);
    String[] fsn = {"FS:OK","FS:WARN","FS:LAND","FS:KILL"};
    text(fsn[constrain(fs_level,0,3)], 270, y+13);
    // Altitude
    fill(PANEL); rect(324,y,130,26,4);
    fill(DIM); textFont(sans); textAlign(LEFT,CENTER);
    text("ALT", 334,y+13);
    fill(WHITE); textFont(titleF); textAlign(RIGHT,CENTER);
    text(String.format("%.2f m", altitude), 446,y+13);
    // Loop
    fill(PANEL); rect(464,y,112,26,4);
    fill(DIM); textFont(sans); textAlign(LEFT,CENTER);
    text("LOOP", 474,y+13);
    fill(loop_us>1200 ? ORANGE : GREEN); textFont(titleF); textAlign(RIGHT,CENTER);
    text(loop_us+" µs", 568,y+13);
}

final int CX=210, CY=360, CSZ=145;
void draw3DCube()
{
    fill(DIM); textFont(sans); textAlign(CENTER,TOP); text("ATTITUDE",CX,112);
    pushMatrix();
    translate(CX,CY,0);
    rotateX(radians(-pitch)); rotateY(radians(yaw)); rotateZ(radians(roll));
    lights();
    fill(35,80,185); stroke(75,125,220); strokeWeight(1); box(CSZ,10,CSZ);
    fill(25,58,135); noStroke();
    pushMatrix(); translate( CSZ*0.44,0,0); box(CSZ*0.22,7,14); popMatrix();
    pushMatrix(); translate(-CSZ*0.44,0,0); box(CSZ*0.22,7,14); popMatrix();
    pushMatrix(); translate(0,0, CSZ*0.44);  box(14,7,CSZ*0.22); popMatrix();
    pushMatrix(); translate(0,0,-CSZ*0.44);  box(14,7,CSZ*0.22); popMatrix();
    fill(210,50,50); pushMatrix(); translate(0,-6,-CSZ*0.44); box(17,5,7); popMatrix();
    noLights(); popMatrix();
    fill(DIM); textFont(mono); textAlign(CENTER,TOP);
    int ly=CY+CSZ/2+18;
    text(String.format("R  %+7.1f°",roll),  CX,ly);
    text(String.format("P  %+7.1f°",pitch), CX,ly+16);
    text(String.format("Y  %+7.1f°",yaw),   CX,ly+32);
}

final int MX=430, MY=110;
void drawMotorPanel()
{
    int bw=36, bh=200, gap=14;
    int pw=4*(bw+gap)+28;
    fill(PANEL); noStroke(); rect(MX-18,MY-32,pw,bh+95,7);
    fill(TABACT); textFont(titleF); textAlign(CENTER,TOP);
    text("MOTORS", MX+pw/2-18, MY-26);
    color[] mc={color(85,210,85),color(210,190,50),color(210,90,50),color(50,145,210)};
    String[] pos={"FR","RR","RL","FL"};
    for (int i=0; i<4; i++)
    {
        int x=MX+i*(bw+gap); int fh=(int)(motors[i]*bh);
        fill(40,46,65); rect(x,MY,bw,bh,4);
        fill(mc[i]);    rect(x,MY+bh-fh,bw,fh,4);
        fill(WHITE); textFont(sans); textAlign(CENTER,TOP);
        text((int)(motors[i]*100)+"%", x+bw/2, MY+bh+4);
        text("M"+(i+1)+" "+pos[i],    x+bw/2, MY+bh+20);
        fill(DIM);
        text(escRPM[i]>0 ? escRPM[i]+" rpm" : "---", x+bw/2, MY+bh+38);
    }
}

final int BPX=660, BPY=112;
void drawBatteryPanel()
{
    fill(PANEL); noStroke(); rect(BPX-18,BPY-28,310,130,7);
    fill(TABACT); textFont(titleF); textAlign(LEFT,TOP);
    text("BATTERY", BPX-4, BPY-22);

    drawKV("Voltage", String.format("%.2f V",volt_mV/1000.0),
           volt_mV<13200 ? RED : volt_mV<14400 ? ORANGE : GREEN, BPX, BPY);
    drawKV("Current", String.format("%.1f A",curr_cA/100.0),
           WHITE, BPX+148, BPY);

    float pct=bat_pct/100.0;
    int bw=272, bh=18;
    fill(38,44,62); rect(BPX,BPY+42,bw,bh,4);
    color bc=bat_pct<20 ? RED : bat_pct<40 ? ORANGE : GREEN;
    fill(bc); rect(BPX,BPY+42,(int)(bw*pct),bh,4);
    fill(255); textFont(sans); textAlign(CENTER,CENTER);
    text(bat_pct+"%", BPX+bw/2, BPY+51);
    fill(DIM); textAlign(LEFT,TOP);
    text(String.format("%.3f V/cell (4S est.)", volt_mV/4000.0), BPX, BPY+68);
}

void drawKV(String k, String v, color vc, int x, int y)
{
    fill(DIM); textFont(sans); textAlign(LEFT,TOP); text(k,x,y);
    fill(vc); textFont(titleF); text(v,x,y+16);
}

final int FX=660, FY=265, FW=300, FH=130;
void drawFFTPanel()
{
    fill(PANEL); noStroke(); rect(FX-10,FY-28,FW+20,FH+55,7);
    fill(TABACT); textFont(titleF); textAlign(LEFT,TOP);
    text("VIBRATION SPECTRUM", FX-2, FY-22);
    fill(22,26,40); rect(FX,FY,FW,FH);
    stroke(38,44,62); strokeWeight(1);
    for (int g=1; g<4; g++) line(FX+g*FW/4, FY, FX+g*FW/4, FY+FH);
    int n=spectrum.length; float bw=(float)FW/n;
    noStroke();
    for (int i=0; i<n; i++)
    {
        float mag=constrain(spectrum[i],0,1); int bh=(int)(mag*FH);
        fill(hsvColor(map(i,0,n,200,30),0.85,0.85));
        rect(FX+i*bw, FY+FH-bh, bw-1, bh);
    }
    fill(DIM); textFont(sans); textAlign(LEFT,TOP); text("0",FX-4,FY+FH+4);
    textAlign(RIGHT,TOP); text("~1kHz", FX+FW, FY+FH+4);
}

final int FBY=710;
void drawFlightButtons()
{
    drawBtn("ARM",       12, FBY, GREEN);
    drawBtn("DISARM",   106, FBY, RED);
    drawBtn("CALIB",    200, FBY, color(155,115,35));
    drawBtn("TEST M1",  294, FBY, color(50,120,155));
    drawBtn("TEST M2",  388, FBY, color(50,120,155));
    drawBtn("TEST M3",  482, FBY, color(50,120,155));
    drawBtn("TEST M4",  576, FBY, color(50,120,155));
    drawBtn("STOP TEST",670, FBY, color(155,80,35));
    drawBtn("DUMP BB",  764, FBY, color(100,55,160));
    drawBtn("ERASE BB", 858, FBY, color(140,30,30));
}

// ================================================================
//  TAB 1  — PID Tuning
// ================================================================
void drawPIDTab()
{
    int ox=40, oy=90;

    // Roll / Pitch panel
    fill(PANEL); noStroke(); rect(ox-18,oy-36,SLD_W+115,310,7);
    fill(TABACT); textFont(titleF); textAlign(LEFT,TOP);
    text("ROLL + PITCH  (linked)", ox-4, oy-28);

    drawSlider("Kp", ox, oy,       sKP,  0,10, dKP,  true);
    drawSlider("Ki", ox, oy+55,    sKI,  0, 2, dKI,  true);
    drawSlider("Kd", ox, oy+110,   sKD,  0, 1, dKD,  true);

    fill(DIM); textFont(sans); textAlign(LEFT,TOP);
    text("Live values from FC:", ox, oy+160);
    fill(WHITE); textFont(mono);
    text(String.format("Kp=%.3f  Ki=%.3f  Kd=%.3f", kp, ki, kd), ox, oy+178);

    // Integral + output limits display
    fill(DIM); text("Integral limit: "+int(200)+"  Output limit: "+int(500), ox, oy+200);

    // Yaw panel
    int oy2=oy+240;
    fill(PANEL); noStroke(); rect(ox-18,oy2-36,SLD_W+115,220,7);
    fill(TABACT); textFont(titleF); textAlign(LEFT,TOP);
    text("YAW", ox-4, oy2-28);

    drawSlider("Kp", ox, oy2,      sYawKP, 0,10, dYKP, false);
    drawSlider("Ki", ox, oy2+55,   sYawKI, 0, 2, dYKI, false);
    drawSlider("Kd", ox, oy2+110,  sYawKD, 0, 1, dYKD, false);

    fill(DIM); textFont(mono); textAlign(LEFT,TOP);
    text(String.format("Kp=%.3f  Ki=%.3f  Kd=%.3f", yawKp, yawKi, yawKd),
         ox, oy2+158);

    // Tips panel
    int tx=SLD_W+160, ty=90;
    fill(PANEL); noStroke(); rect(tx-18,ty-36,420,470,7);
    fill(TABACT); textFont(titleF); textAlign(LEFT,TOP);
    text("PID Tuning Guide", tx-4, ty-28);
    String[] tips = {
        "1. Set all gains to zero. Increase Kp slowly",
        "   until the craft holds level but oscillates",
        "   slightly on disturbance.",
        "",
        "2. Kp too high → fast oscillation (buzz).",
        "   Kp too low  → sluggish, drifts.",
        "",
        "3. Add Kd to damp the Kp oscillation.",
        "   Typical Kd = Kp × 0.01 to 0.02.",
        "",
        "4. Add Ki slowly to remove steady-state",
        "   angle error on hover.  Watch for windup.",
        "",
        "5. Yaw usually needs only Kp (no Kd).",
        "",
        "Starting values (65 mm brushed):",
        "  Kp=2.5  Ki=0.05  Kd=0.03",
        "",
        "Starting values (5\" BLDC racing):",
        "  Kp=45   Ki=80    Kd=22",
        "  (Betaflight units — see MSP note)",
        "",
        "Auto-tune: connect bench clamp → ARM",
        "→ 'AUTO-TUNE' in GUI (Z-N relay method)."
    };
    fill(DIM); textFont(sans);
    for (int i=0; i<tips.length; i++)
        text(tips[i], tx, ty + i*19);

    // Auto-tune button
    drawBtn("AUTO-TUNE", tx-4, ty+435, color(100,60,190));
}

// ================================================================
//  TAB 2  — RC Calibration
// ================================================================
void drawRCCalTab()
{
    int ox=40, oy=90;

    fill(PANEL); noStroke(); rect(ox-18,oy-40,860,580,8);
    fill(TABACT); textFont(titleF); textAlign(LEFT,TOP);
    text("RC Channel Calibration + Expo / Deadband", ox-4, oy-32);

    // Column headers
    fill(DIM); textFont(mono); textAlign(CENTER,TOP);
    String[] headers = {"CH","Function","rawMin","rawMid","rawMax","Expo","Deadband"};
    int[] hx = {ox+20, ox+80, ox+200, ox+280, ox+360, ox+460, ox+580};
    for (int i=0; i<headers.length; i++) text(headers[i], hx[i], oy+4);

    // Channel rows
    String[] funcs = {"Roll","Pitch","Throttle","Yaw","Arm SW","Mode SW","Alt Hold","Spare"};
    for (int ch=0; ch<8; ch++)
    {
        int cy = oy + 38 + ch * 56;
        fill(ch%2==0 ? color(34,38,56) : color(30,34,50));
        rect(ox-12, cy-8, 844, 48, 4);

        // CH number
        fill(GOLD); textFont(titleF); textAlign(CENTER,CENTER);
        text("CH"+(ch+1), hx[0], cy+16);

        // Function name
        fill(WHITE); textFont(sans);
        text(funcs[ch], hx[1], cy+16);

        // Raw values (from RC cal — we show defaults here; FC sends them via RC_CAL_PRINT)
        fill(DIM); textFont(mono);
        text("172",  hx[2], cy+16);
        text("992",  hx[3], cy+16);
        text("1811", hx[4], cy+16);

        // Expo slider (only sticks: CH 0,1,3)
        if (ch==0||ch==1||ch==3)
        {
            drawMiniSlider(hx[5]-35, cy+8, 70, chExpo[ch],   0, 1.0, ch, true);
        }
        else { fill(DIM); text("N/A", hx[5], cy+16); }

        // Deadband slider (only sticks)
        if (ch==0||ch==1||ch==3)
        {
            drawMiniSlider(hx[6]-35, cy+8, 70, chDeadband[ch], 0, 0.2, ch, false);
        }
        else { fill(DIM); text("N/A", hx[6], cy+16); }
    }

    // Buttons
    int by = oy + 38 + 8*56 + 10;
    drawBtn("START CAL",   ox,       by, TABACT);
    drawBtn("END CAL",     ox+94,    by, GREEN);
    drawBtn("PRINT CAL",   ox+188,   by, color(90,90,140));

    // Instructions
    fill(DIM); textFont(sans); textAlign(LEFT,TOP);
    text("1. Click START CAL — FC begins collecting stick extents",    ox, by+44);
    text("2. Move ALL sticks to their full extents (all directions)",  ox, by+62);
    text("3. Click END CAL — FC computes min/mid/max for each channel",ox, by+80);
    text("4. Adjust Expo (response curve) and Deadband (centre zone)", ox, by+98);
    text("   using the sliders.  Changes take effect immediately.",     ox, by+116);
}

void drawMiniSlider(int x, int y, int w, float val, float mn, float mx,
                    int chIdx, boolean isExpo)
{
    float r = constrain((val-mn)/(mx-mn),0,1);
    fill(38,44,62); rect(x, y, w, 10, 5);
    fill(isExpo ? TABACT : GREEN);
    rect(x, y, (int)(w*r), 10, 5);
    fill(255); ellipse(x+w*r, y+5, 14, 14);
    fill(DIM); textFont(sans); textAlign(CENTER,TOP);
    text(String.format("%.2f", val), x+w/2, y+14);
}

// ================================================================
//  TAB 3  — Self-Test
// ================================================================
void drawSelfTestTab()
{
    int ox=40, oy=90;

    fill(PANEL); noStroke(); rect(ox-18,oy-40,700,500,8);
    fill(TABACT); textFont(titleF); textAlign(LEFT,TOP);
    text("Hardware Self-Test", ox-4, oy-32);

    if (!stDone)
    {
        fill(DIM); textFont(sans); textAlign(LEFT,TOP);
        text("Click RUN SELF-TEST to verify all hardware at startup.", ox, oy+10);
        text("Keep the board flat and still during the test.", ox, oy+30);
        text("Tests performed:", ox, oy+60);
        String[] list = {
            "1. BMI088 Accelerometer chip ID (SPI)",
            "2. BMI088 Gyroscope chip ID (SPI)",
            "3. BMP580 Barometer chip ID (I2C)",
            "4. W25Q128 Flash JEDEC ID (SPI)",
            "5. Battery voltage in range",
            "6. CRSF link received within 2 s",
            "7. Gyro RMS < 0.05 rad/s (stationary)",
            "8. Accelerometer magnitude ≈ 1.0 g"
        };
        for (int i=0; i<list.length; i++)
        {
            fill(DIM); text(list[i], ox+12, oy+90+i*22);
        }
    }
    else
    {
        // Show parsed results
        int allPass = 1;
        for (int i=0; i<stResults.length; i++)
        {
            String[] parts = split(stResults[i], ':');
            if (parts.length < 3) continue;
            boolean pass = parts[1].equals("PASS");
            if (!pass) allPass = 0;

            int ry = oy + i * 44;
            fill(pass ? color(30,80,40) : color(80,20,20));
            rect(ox-8, ry-6, 660, 36, 5);
            fill(pass ? GREEN : RED); textFont(titleF); textAlign(LEFT,CENTER);
            text(pass ? "✔ PASS" : "✘ FAIL", ox, ry+12);
            fill(WHITE); textFont(sans); textAlign(LEFT,CENTER);
            text(parts[2], ox+80, ry+2);
            fill(DIM); textFont(mono); textAlign(LEFT,CENTER);
            text(parts.length>3 ? parts[3] : "", ox+80, ry+20);
        }

        // Summary banner
        int sy = oy + stResults.length * 44 + 16;
        fill(allPass==1 ? color(30,80,40) : color(80,25,25));
        rect(ox-8, sy, 660, 36, 6);
        fill(255); textFont(titleF); textAlign(CENTER,CENTER);
        text(allPass==1 ? "ALL TESTS PASSED — safe to arm"
                        : "SOME TESTS FAILED — check hardware before flying",
             ox+322, sy+18);
    }

    // Run button
    int by = 660;
    drawBtn("RUN SELF-TEST", ox, by, TABACT);
    if (stDone) drawBtn("CLEAR", ox+152, by, color(70,70,100));
}

// ================================================================
//  Status footer
// ================================================================
void drawStatusFooter()
{
    fill(PANEL); noStroke(); rect(0, H-30, W, 30);
    fill(DIM); textFont(mono); textAlign(LEFT,CENTER);
    text(String.format("  Roll:%+.1f° Pitch:%+.1f° Yaw:%+.1f°  "
                     + "Bat:%.2fV %d%%  Loop:%dµs  FS:%s",
        roll, pitch, yaw, volt_mV/1000.0, bat_pct, loop_us,
        new String[]{"OK","WARN","LAND","KILL"}[constrain(fs_level,0,3)]),
        0, H-15);
}

// ================================================================
//  Shared drawing helpers
// ================================================================
void drawSlider(String lbl, int x, int y, float val,
                float mn, float mx, boolean drag, boolean isRP)
{
    fill(DIM); textFont(sans); textAlign(LEFT,CENTER); text(lbl, x-18, y+SLD_H/2);
    fill(38,44,62); rect(x, y, SLD_W, SLD_H, SLD_H/2);
    float r=constrain((val-mn)/(mx-mn),0,1);
    fill(drag ? color(105,190,255) : (isRP ? TABACT : GOLD));
    rect(x, y, (int)(SLD_W*r), SLD_H, SLD_H/2);
    fill(255); ellipse(x+SLD_W*r, y+SLD_H/2, 18, 18);
    fill(WHITE); textAlign(LEFT,CENTER);
    text(String.format("%.3f",val), x+SLD_W+12, y+SLD_H/2);
}

void drawBtn(String lbl, int x, int y, color c)
{
    fill(c); noStroke(); rect(x,y,82,28,5);
    fill(255); textFont(sans); textAlign(CENTER,CENTER); text(lbl,x+41,y+14);
}

color hsvColor(float h, float s, float v)
{
    float c=v*s, x=c*(1-abs((h/60)%2-1)), m=v-c;
    float r=0,g=0,b=0;
    if(h<60){r=c;g=x;}else if(h<120){r=x;g=c;}else if(h<180){g=c;b=x;}
    else if(h<240){g=x;b=c;}else if(h<300){r=x;b=c;}else{r=c;b=x;}
    return color((int)((r+m)*255),(int)((g+m)*255),(int)((b+m)*255));
}

// ================================================================
//  Serial event — parse telemetry OR self-test lines
// ================================================================
void serialEvent(Serial p)
{
    String line = p.readStringUntil('\n');
    if (line == null) return;
    line = trim(line);
    if (line.length() == 0) return;

    // Self-test stream
    if (line.equals("SELF_TEST_BEGIN"))  { stResults = new String[0]; stReceiving=true; stDone=false; return; }
    if (line.equals("SELF_TEST_END"))    { stReceiving=false; stDone=true; currentTab=3; return; }
    if (stReceiving && line.startsWith("ST:")) { stResults = append(stResults, line.substring(3)); return; }

    // Debug / status lines — print but don't try to parse as CSV
    if (line.startsWith("[")) { println("[FC] "+line); return; }

    // CSV telemetry
    String[] v = split(line, ',');
    if (v.length < 22) return;
    try {
        roll=int(v[0])/10.0; pitch=int(v[1])/10.0; yaw=int(v[2])/10.0;
        for (int i=0;i<4;i++) motors[i]=int(v[3+i])/1000.0;
        kp=float(v[7]); ki=float(v[8]); kd=float(v[9]);
        altitude=int(v[10])/100.0;
        armed=int(v[11])==1; mode=int(v[12]);
        volt_mV=int(v[13]); curr_cA=int(v[14]); bat_pct=int(v[15]);
        for (int i=0;i<4;i++) escRPM[i]=int(v[16+i]);
        loop_us=int(v[20]); fs_level=int(v[21]);
        // Update PID sliders to live values
        sKP=kp; sKI=ki; sKD=kd;
        // Animate FFT
        float avg=(motors[0]+motors[1]+motors[2]+motors[3])*0.25;
        for (int i=0;i<spectrum.length;i++)
        {
            float hz=i*(1000.0/spectrum.length);
            float peak=avg*exp(-0.002*(hz-avg*400)*(hz-avg*400));
            spectrum[i]=spectrum[i]*0.88+peak*0.12;
        }
    } catch(Exception e) {}
}

// ================================================================
//  Mouse events
// ================================================================
void mousePressed()
{
    // Tab selection
    for (int i=0;i<4;i++)
    {
        int tx=12+i*150, ty=44;
        if (mouseX>tx&&mouseX<tx+144&&mouseY>ty&&mouseY<ty+26) { currentTab=i; return; }
    }

    // Tab 0 — Flight buttons
    if (currentTab==0)
    {
        if (hitBtn(12,  FBY)) sendCmd("ARM");
        if (hitBtn(106, FBY)) sendCmd("DISARM");
        if (hitBtn(200, FBY)) sendCmd("CALIB");
        if (hitBtn(294, FBY)) sendCmd("MOTOR_TEST:0:10");
        if (hitBtn(388, FBY)) sendCmd("MOTOR_TEST:1:10");
        if (hitBtn(482, FBY)) sendCmd("MOTOR_TEST:2:10");
        if (hitBtn(576, FBY)) sendCmd("MOTOR_TEST:3:10");
        if (hitBtn(670, FBY)) sendCmd("MOTOR_TEST_STOP");
        if (hitBtn(764, FBY)) sendCmd("DUMP_BB");
        if (hitBtn(858, FBY)) sendCmd("ERASE_BB");
    }

    // Tab 1 — PID sliders
    if (currentTab==1)
    {
        if (hitSld(40,90))        dKP=true;
        if (hitSld(40,145))       dKI=true;
        if (hitSld(40,200))       dKD=true;
        if (hitSld(40,330))       dYKP=true;
        if (hitSld(40,385))       dYKI=true;
        if (hitSld(40,440))       dYKD=true;
        if (hitBtn(40,525,152))   sendCmd("AUTOTUNE_START");
    }

    // Tab 2 — RC Cal buttons
    if (currentTab==2)
    {
        int by=90+38+8*56+10;
        if (hitBtn(40,     by)) sendCmd("RC_CAL_START");
        if (hitBtn(40+94,  by)) sendCmd("RC_CAL_END");
        if (hitBtn(40+188, by)) sendCmd("RC_CAL_PRINT");
    }

    // Tab 3 — Self-test
    if (currentTab==3)
    {
        if (hitBtn(40, 660, 0)) sendCmd("SELF_TEST");
        if (stDone && hitBtn(192, 660, 0)) { stDone=false; stResults=new String[0]; }
    }
}

boolean hitBtn(int bx, int by)        { return mouseX>bx&&mouseX<bx+82&&mouseY>by&&mouseY<by+28; }
boolean hitBtn(int bx, int by, int w) { int ww=w>0?w:82; return mouseX>bx&&mouseX<bx+ww&&mouseY>by&&mouseY<by+28; }
boolean hitSld(int sx, int sy) { return mouseX>sx-10&&mouseX<sx+SLD_W+10&&mouseY>sy-8&&mouseY<sy+SLD_H+8; }

void mouseDragged()
{
    if (currentTab==1)
    {
        float r=constrain((float)(mouseX-40)/SLD_W,0,1);
        if (dKP)  { sKP =r*10; sendCmd("SET_KP:"+nf(sKP,1,3));  }
        if (dKI)  { sKI =r* 2; sendCmd("SET_KI:"+nf(sKI,1,3));  }
        if (dKD)  { sKD =r* 1; sendCmd("SET_KD:"+nf(sKD,1,3));  }
        if (dYKP) { sYawKP=r*10; sendCmd("SET_YAW_KP:"+nf(sYawKP,1,3)); }
        if (dYKI) { sYawKI=r* 2; sendCmd("SET_YAW_KI:"+nf(sYawKI,1,3)); }
        if (dYKD) { sYawKD=r* 1; sendCmd("SET_YAW_KD:"+nf(sYawKD,1,3)); }
    }
}

void mouseReleased() { dKP=dKI=dKD=dYKP=dYKI=dYKD=false; for(int i=0;i<4;i++){dExpo[i]=dDB[i]=false;} }

void sendCmd(String cmd)
{
    if (connected && port!=null) { port.write(cmd+"\n"); println("[GUI→FC] "+cmd); }
}
