import ipcapture.*;
import processing.video.*;
import gab.opencv.*;
import KinectPV2.*;
import java.util.LinkedList;

IPCapture cam;
PImage img;  
float strength=2.0;
float zoom=1.0;

float headX =0;
float headY=175;
int headZ;

PShader texShader;

KinectPV2 kinect;
OpenCV opencv;

float polygonFactor = 1;
int threshold = 10;

float maxD = 1.25f;
float minD = 0.5f;
int framesAvgX=0;
int framesAvgZ=0;
int framesAvgY=0;

float x0, x1, y0, y1;
float panx = 0;
float panZ = 0;
float headZoom;
float headDepth;

boolean contourBodyIndex = false;

LinkedList<Integer> xPtsQe=new LinkedList<Integer>();
LinkedList<Integer> yPtsQe=new LinkedList<Integer>();
LinkedList<Integer> zPtsQe=new LinkedList<Integer>();

ArrayList<Contour> contours;

int [] colorRaw;
int [] depthRaw;

PImage pointDepthRaw;

boolean running =true;

void setup() {
  frameRate(10000);
  size(1920, 1080, P3D);

  int halfWidth=width/2;
  int halfHeight=height/2;

  float correctionRadius = sqrt(pow(1920, 2) + pow(1080, 2)) / strength;
  float theta;

  texShader = loadShader("texfrag.glsl");
  textureMode(NORMAL);
  texShader.set("correctRadius", correctionRadius);
  texShader.set("zoom", zoom);
  texShader.set("xRes", (float)width);
  texShader.set("yRes", (float)height);
  
  opencv = new OpenCV(this, 512, 424);
  kinect = new KinectPV2(this);
  kinect.enablePointCloud(true);
  kinect.activateRawDepth(true);
  kinect.activateRawColor(true);
  kinect.enableBodyTrackImg(true);
  
  img = loadImage("barreled.jpg");
  cam = new IPCapture(this, "http://136.244.17.215//axis-cgi/mjpg/video.cgi", "chris", "virtualwindow");
  cam.start();
  kinect.init();
  
  thread("posFinder");
}

void draw() {
  if (cam.isAvailable()) {
    cam.read();
    texShader.set("tex", cam);

    background(0);  
    shader(texShader);
    //zoom
    if (headY>318) {
      headZoom= 2;
    } else if (headY<106) {
      headZoom= 1;
    } else {
      headZoom=1+(headY-106)/212;
    }
    
    // apply panning and zooming
    x0 = 0.5*width-(width*(headZoom)/2);
    x1 = x0 + width*headZoom; //right
    y0 = 0.5*height-(height*(headZoom)/2); //top
    y1 = y0 + height*headZoom; //bottom
    
    if ((x0 + panx) > 0.0f) {
      x0 = 0.0f;
      x1 = x0 + width*headZoom;
    }
    else if ((x1 + panx) < 1920){
      x1 = width;
      x0 = x1 - width*headZoom;
    }
    else {
      x0 += panx;
      x1 = x0 + width*headZoom;
    }
    
    if ((y0 + panZ) > 0.0f) {
      y0 = 0.0f;
      y1 = y0 + height*headZoom;
    }
    else if ((y1 + panZ) < 1080){
      y1 = height;
      y0 = y1 - height*headZoom;
    } 
    else {
      y0 += panZ;
      y1 = y0 + height*headZoom;
    }
    
    beginShape(QUADS);
    normal(0, 0, 1);

    vertex(x0, y1, 0, 1);
    vertex(x1, y1, 1, 1);
    vertex(x1, y0, 1, 0);
    vertex(x0, y0, 0, 0);
    endShape();  
  }
  //println(frameRate);
}

void keyPressed() {
  if (key == 's') {
    headY++;
  }
  if (key == 'w') {
    if (headY>1) {
      headY=headY-1;
    }
  }
  if (key == CODED) {
    if (keyCode == LEFT) {
      panx += -0.6f;
    } 
    if (keyCode == RIGHT) {
      panx += 0.6f;
    }
    if (keyCode == UP) {
      panZ += 0.6f;
    }
    if (keyCode == DOWN) {
      panZ += -0.6f;
    }
  } 
}

void stop() {
  running = false;
  super.stop();
}

void posFinder(){
  while (running) {
    colorRaw = kinect.getRawColor();
    depthRaw = kinect.getRawDepth();
    pointDepthRaw = kinect.getPointCloudDepthImage();
    
    if (contourBodyIndex) {
      opencv.loadImage(kinect.getBodyTrackImage());
      opencv.gray();
      opencv.threshold(threshold);
      PImage dst = opencv.getOutput();
    } else {
      opencv.loadImage(kinect.getPointCloudDepthImage());
      opencv.gray();
      opencv.threshold(threshold);
      PImage dst = opencv.getOutput();
    }
    maxD=1.26;
  kinect.setLowThresholdPC(minD);
  kinect.setHighThresholdPC(maxD);
  ArrayList<Contour> contours = opencv.findContours(false, false);
  
  if (contours.size() > 0) {
    float largest_size=0;
    for (int i=0;i<contours.size();i++) {
      float c_size=contours.get(i).numPoints();
      
      contours.get(i).setPolygonApproximationFactor(polygonFactor);
      if (contours.get(i).numPoints() > 250) {
        int xvals=0;
        int yvals=0;
        int points=0;
        int avgX=0;
        int avgY=0;
        int tX=0;
        int tY=0;
        
        for (PVector point : contours.get(i).getPolygonApproximation ().getPoints()) {
            points++;
            xvals+=point.x;
            yvals+=point.y;
        }
     
        avgX=xvals/points;
        avgY=yvals/points;
        float headDepth=pointDepthRaw.get(avgX,avgY)/16777215;
          if(headDepth<0){
            headDepth=headDepth+256;
          }
        float roughHeadAbs=maxD*(headDepth/256);
        maxD=roughHeadAbs+.1;
      }
    }
  }
  kinect.setLowThresholdPC(minD);
  kinect.setHighThresholdPC(maxD);
  println("maxD", maxD);
  
  
    contours = opencv.findContours(false, false);
    if (contours.size() > 0) {
      int largest_i=0;
      float largest_size=0;
      for (int i=0;i<contours.size();i++) {
        float c_size=contours.get(i).numPoints();
        if (c_size>largest_size){
          largest_i=i;
          largest_size=c_size;
        }
      }
      contours.get(largest_i).setPolygonApproximationFactor(polygonFactor);
      if (contours.get(largest_i).numPoints() > 250) {
        //println(contours.get(largest_i).numPoints());
        int xvals=0;
        int yvals=0;
        int points=0;
        int avgX=0;
        int avgY=0;
        int tX=0;
        int tY=0;
        int tZ=0;
        for (PVector point : contours.get(largest_i).getPolygonApproximation ().getPoints()) {
            points++;
            xvals+=point.x;
            yvals+=point.y;
        }
        //dynamic clipping
        avgX=xvals/points;
        avgY=yvals/points;
        xPtsQe.add(avgX);
        yPtsQe.add(avgY);
        framesAvgX+=avgX;
        framesAvgY+=avgY;
        if (yPtsQe.size()>20){
          tX=xPtsQe.removeFirst();
          tY=yPtsQe.removeFirst();
          framesAvgX-=tX;
          framesAvgY-=tY;
        }
        headY=framesAvgY/yPtsQe.size();
        panx=framesAvgX/xPtsQe.size()-256;
        headDepth=pointDepthRaw.get(framesAvgX/xPtsQe.size(),framesAvgY/yPtsQe.size())/16777215;
        if(headDepth<0){
          headDepth=headDepth+256;
        }
        headZ=int(headDepth);
        zPtsQe.add(headZ);
        framesAvgZ+=headZ;
        if (zPtsQe.size()>20){
          tZ=zPtsQe.removeFirst();
          framesAvgZ-=tZ;
        }
        panZ=(framesAvgZ/zPtsQe.size())-127;
        }
        else{
          maxD=1.25;}
    }
    kinect.setLowThresholdPC(minD);
    kinect.setHighThresholdPC(maxD);
  }
}

