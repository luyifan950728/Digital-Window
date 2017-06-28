import ipcapture.*;
import processing.video.*;
import gab.opencv.*;
import KinectPV2.*;
import java.util.LinkedList;
import java.nio.FloatBuffer;

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

float preMaxD;
float maxD = 1.25f;
float minD = 0.5f;
int framesAvgX=0;
float framesAvgZ=0;
int framesAvgY=0;

float x0, x1, y0, y1;
float panx = 0;
float panZ = 0;
float oldPanZ;
float headZoom;
float headDepth;

boolean contourBodyIndex = false;

LinkedList<Integer> xPtsQe=new LinkedList<Integer>();
LinkedList<Integer> yPtsQe=new LinkedList<Integer>();
LinkedList<Float> zPtsQe=new LinkedList<Float>();

ArrayList<Contour> contours;

PImage pointDepthRaw;
//values used to test framerate
int start;
int counter=0;
int end;
int time;

//boolean running =true;

void setup() {
  //set high theoretical framerate to prevent bottlenecking
  frameRate(10000);
  size(1920, 1080, P3D);
  
  //sets texture for barrel correction
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
  
  //initiate openCV and kinect
  opencv = new OpenCV(this, 512, 424);
  kinect = new KinectPV2(this);
  kinect.enablePointCloud(true);
  kinect.activateRawDepth(true);
  kinect.activateRawColor(true);
  kinect.enableBodyTrackImg(true);
  
  //IPCapture opject can be manipulated just like a pimage
  //these login credentials will remain identical, but the IP address may change
  //use axis iputility to find axis cam IPs on the network
  cam = new IPCapture(this, "http://136.244.17.215//axis-cgi/mjpg/video.cgi", "chris", "virtualwindow");
  cam.start();
  kinect.init();
  
  //thread("posFinder");
  //start value used to determine frames per second over time
  //start=millis()+1000*(second()+60*(minute()+60*(hour()+24*day())));
}

void draw() {
  //counter used to count FPS
  //counter++;
  if (cam.isAvailable()) {
    // code from kinectPV2 library example code 'OpenCV_Processing'
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
    //sets arbitrary depth threshold
    maxD=1.26;
  kinect.setLowThresholdPC(minD);
  kinect.setHighThresholdPC(maxD);
  //finds contours and determines the largest if there is one
  //this is the preliminary loop used to find the rough depth value of the head for dynamic thresholding
  ArrayList<Contour> contours = opencv.findContours(false, false);
  if (contours.size() > 0) {
    int largest_i=0;
    float largest_size=0;
    for (int i=0; i<contours.size (); i++) {
      float c_size=contours.get(i).numPoints();
      //size 600 is arbitrary, but used to filter out ambient objects like screen and table
      if (c_size>largest_size && c_size<600) {
        largest_i=i;
        largest_size=c_size;
      }
    }
    contours.get(largest_i).setPolygonApproximationFactor(polygonFactor);
    
    if (contours.get(largest_i).numPoints() > 200) {
      int xvals=0;
      int yvals=0;
      int points=0;
      int avgX=0;
      int avgY=0;
      int tX=0;
      int tY=0;
      //calculates average of all points in opencv contour
      for (PVector point : contours.get (largest_i).getPolygonApproximation ().getPoints()) {
        points++;
        xvals+=point.x;
        yvals+=point.y;
      }

      avgX=xvals/points;
      avgY=yvals/points;
      //useful code from prof lee
      
      // James
      // mapped from (0.0 - 8.0) to gray color (0-255)
      // just use one channel value to restore real height
      // for explanation, refer to PointCloudDepth example app
      // headDepth seems in feet
      //float headDepth = red(pointDepthRaw.pixels[avgX + avgY*512]) / 255 * 8.0;
      //println("head depth :", headDepth);
      
      // James: access 3D position
      // refer to PointCloudOGL example
      // x, y, z values are in meter and seem better than above calculation
      FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();
      //float hx = pointCloudBuffer.get( (avgX + avgY*512)*3 );
      //float hy = pointCloudBuffer.get( (avgX + avgY*512)*3 + 1);
      float hz = pointCloudBuffer.get( (avgX + avgY*512)*3 + 2);
      //println("float buffer: ", hx, hy, hz );
      
      //sets depth threshold .1 meters below estimated top of head
      maxD=hz+.10;
    }
  }
  //.6 is arbitrary, but this code ensures that the threshold is 'reasonable'
  if (maxD<.6 ) {
    maxD=preMaxD;
  } 
  //sets new depth max threshold
  kinect.setHighThresholdPC(maxD);
  //find contours based on new depth
  contours = opencv.findContours(false, false);
  //this loop is almost identical to the one above
  if (contours.size() > 0) {
    int largest_i=0;
    float largest_size=0;
    for (int i=0; i<contours.size (); i++) {
      float c_size=contours.get(i).numPoints();
      if (c_size>largest_size && c_size<600 ) {
        largest_i=i;
        largest_size=c_size;
      }
    }
    contours.get(largest_i).setPolygonApproximationFactor(polygonFactor);
    if (contours.get(largest_i).numPoints() > 200) {
      int xvals=0;
      int yvals=0;
      int points=0;
      int avgX=0;
      int avgY=0;
      int tX=0;
      float tZ=0;
      int tY=0;
      

      for (PVector point : contours.get (largest_i).getPolygonApproximation ().getPoints()) {
        points++;
        xvals+=point.x;
        yvals+=point.y;
      }
      //uses a queue to store 15 most recent frames of x and y values, averaging them to smooth image. 
      //removes oldest ones from sum when they're removed from queue
      avgX=xvals/points;
      avgY=yvals/points;
      xPtsQe.add(avgX);
      yPtsQe.add(avgY);
      framesAvgX+=avgX;
      framesAvgY+=avgY;
      if (yPtsQe.size()>15) {
        tX=xPtsQe.removeFirst();
        tY=yPtsQe.removeFirst();
        framesAvgX-=tX;
        framesAvgY-=tY;
      }
      headY=framesAvgY/yPtsQe.size();
      //pan set minus 256 because 512 pixels wide, pan is centered at 0/ center of screen
      panx=framesAvgX/xPtsQe.size()-256;
      
      //averages depth value based on 3x3 pixel square surrounding center of head
      float zsum=0.0;
      int divisor=0;
      FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();
      //checks that values are valid, divisor only adds if the values are good
      for(int i=-1;i<2;i++){
        for(int j=-1;j<2;j++){
          //.5 arbitrary value to make sure depth is valid
          if (pointCloudBuffer.get(((framesAvgX/xPtsQe.size())+i + (framesAvgY/yPtsQe.size()*512)+j)*3 + 2)>.5){
            divisor+=1;
            zsum+=pointCloudBuffer.get(((framesAvgX/xPtsQe.size())+i + (framesAvgY/yPtsQe.size()*512)+j)*3 + 2);
          }
        }
      }
      float hz=0;
      panZ=0;
      //if data is valid, averages z values over 15 frames
      if(divisor!=0){
         hz=zsum/divisor;
         zPtsQe.add(hz);
         framesAvgZ+=hz;
          if (zPtsQe.size()>15) {
            tZ=zPtsQe.removeFirst();
            framesAvgZ-=tZ;
          }
         hz=framesAvgZ/zPtsQe.size();
         //panZ calculation needs work, but seeks to center up and down pan at middle of screen and scale appropriately
         panZ=3*(hz*200-200);
        oldPanZ=panZ;
      }
      //if data not valid, set to previous frame's
      else{panZ=oldPanZ;}

    }
  }
  //reads camera image and applies barrel correction
    cam.read();
    texShader.set("tex", cam);

    background(0);  
    shader(texShader);
    //zoom
    //this code may not work correctly, as last minute adjustments were made because the zooming was inverted
    //image should zoom out as head moves closer
    if (headY>318) {
      headZoom= 1;
    } else if (headY<106) {
      headZoom= 2;
    } else {
      headZoom=1.5-(headY-106)/212;
      
      //headZoom=1-(headY-106)/212;
    }

    // apply panning and zooming
    //right now these have a one to one relationship with pixel position
    //used to determine how opengl object is drawn below
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
    //draws cam image as opengl object
    beginShape(QUADS);
    normal(0, 0, 1);

    vertex(x0, y1, 0, 1);
    vertex(x1, y1, 1, 1);
    vertex(x1, y0, 1, 0);
    vertex(x0, y0, 0, 0);
    endShape();  
  }
}

void keyPressed() {
  //x keypress used to terminate program and take timestamps when seeking FPS
//  if (key == 'x'){
//    end=millis()+1000*(second()+60*(minute()+60*(hour()+24*day())));
//    time=end-start;
//    println(time);
//    println(counter/(time*.001));
//    noLoop();
//  }
//old code for manual head position simulation, kept because it might be useful
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
//******************
//below code is for multithread, more updated version of this in kinect_static_image
//*******************

//
//void stop() {
//  running = false;
//  super.stop();
//}

//void posFinder(){
//  while (running) {
//    if (contourBodyIndex) {
//      opencv.loadImage(kinect.getBodyTrackImage());
//      opencv.gray();
//      opencv.threshold(threshold);
//      PImage dst = opencv.getOutput();
//    } else {
//      opencv.loadImage(kinect.getPointCloudDepthImage());
//      opencv.gray();
//      opencv.threshold(threshold);
//      PImage dst = opencv.getOutput();
//    }
//    maxD=1.26;
//      kinect.setLowThresholdPC(minD);
//  kinect.setHighThresholdPC(maxD);
//  ArrayList<Contour> contours = opencv.findContours(false, false);
//  if (contours.size() > 0) {
//    int largest_i=0;
//    float largest_size=0;
//    for (int i=0; i<contours.size (); i++) {
//      float c_size=contours.get(i).numPoints();
//      if (c_size>largest_size && c_size<600) {
//        //check that contour is in proper region of screen
//        largest_i=i;
//        largest_size=c_size;
//      }
//    }
//    contours.get(largest_i).setPolygonApproximationFactor(polygonFactor);
//    if (contours.get(largest_i).numPoints() > 200) {
//      int xvals=0;
//      int yvals=0;
//      int points=0;
//      int avgX=0;
//      int avgY=0;
//      int tX=0;
//      int tY=0;
//
//      for (PVector point : contours.get (largest_i).getPolygonApproximation ().getPoints()) {
//        points++;
//        xvals+=point.x;
//        yvals+=point.y;
//      }
//
//      avgX=xvals/points;
//      avgY=yvals/points;
//
//      // James
//      // mapped from (0.0 - 8.0) to gray color (0-255)
//      // just use one channel value to restore real height
//      // for explanation, refer to PointCloudDepth example app
//      // headDepth seems in feet
//      float headDepth = red(pointDepthRaw.pixels[avgX + avgY*512]) / 255 * 8.0;
//      println("head depth :", headDepth);
//      
//      // James: access 3D position
//      // refer to PointCloudOGL example
//      // x, y, z values are in meter and seem better than above calculation
//      FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();
//      float hx = pointCloudBuffer.get( (avgX + avgY*512)*3 );
//      float hy = pointCloudBuffer.get( (avgX + avgY*512)*3 + 1);
//      float hz = pointCloudBuffer.get( (avgX + avgY*512)*3 + 2);
//      println("float buffer: ", hx, hy, hz );
//      
//      maxD=hz+.10;
//    }
//  }
//
//  if (maxD<.6 ) {
//    maxD=preMaxD;
//  } 
//  //kinect.setLowThresholdPC(minD);
//  kinect.setHighThresholdPC(maxD);
//  contours = opencv.findContours(false, false);
//  if (contours.size() > 0) {
//    int largest_i=0;
//    float largest_size=0;
//    for (int i=0; i<contours.size (); i++) {
//      float c_size=contours.get(i).numPoints();
//      if (c_size>largest_size && c_size<600 ) {
//        largest_i=i;
//        largest_size=c_size;
//      }
//    }
//    contours.get(largest_i).setPolygonApproximationFactor(polygonFactor);
//    if (contours.get(largest_i).numPoints() > 200) {
//      println(contours.get(largest_i).numPoints());
//      int xvals=0;
//      int yvals=0;
//      int points=0;
//      int avgX=0;
//      int avgY=0;
//      int tX=0;
//      int tY=0;
//      
//      stroke(0, 200, 200);
//      beginShape();
//
//      for (PVector point : contours.get (largest_i).getPolygonApproximation ().getPoints()) {
//        vertex(point.x + 512*2, point.y);
//        points++;
//        xvals+=point.x;
//        yvals+=point.y;
//      }
//      endShape();    
//      //dynamic clipping
//      avgX=xvals/points;
//      avgY=yvals/points;
//      xPtsQe.add(avgX);
//      yPtsQe.add(avgY);
//      framesAvgX+=avgX;
//      framesAvgY+=avgY;
//      if (yPtsQe.size()>15) {
//        tX=xPtsQe.removeFirst();
//        tY=yPtsQe.removeFirst();
//        framesAvgX-=tX;
//        framesAvgY-=tY;
//      }
//        headY=framesAvgY/yPtsQe.size();
//        panx=framesAvgX/xPtsQe.size()-256;
//      //println(framesAvgX/xPtsQe.size()+ 512*2,framesAvgY/yPtsQe.size());
//      float zsum=0.0;
//      FloatBuffer pointCloudBuffer = kinect.getPointCloudDepthPos();
//      for(int i=-1;i<2;i++){
//        for(int j=-1;j<2;j++){
//          zsum+=pointCloudBuffer.get(((framesAvgX/xPtsQe.size())+i + (framesAvgY/yPtsQe.size()*512)+j)*3 + 2);
//        }
//      }
//          
//      float hz=zsum/9;
//      println("second loop: ", hz );
//      point(framesAvgX/xPtsQe.size()+ 512, framesAvgY/yPtsQe.size());
//      int headDepth=pointDepthRaw.get(framesAvgX/xPtsQe.size(), framesAvgY/yPtsQe.size())/16777215;
//
//    }
//  }
//  }
//}
//  kinect.setLowThresholdPC(minD);
//  kinect.setHighThresholdPC(maxD);
//  ArrayList<Contour> contours = opencv.findContours(false, false);
//  
//  if (contours.size() > 0) {
//    float largest_size=0;
//    for (int i=0;i<contours.size();i++) {
//      float c_size=contours.get(i).numPoints();
//      
//      contours.get(i).setPolygonApproximationFactor(polygonFactor);
//      if (contours.get(i).numPoints() > 250) {
//        int xvals=0;
//        int yvals=0;
//        int points=0;
//        int avgX=0;
//        int avgY=0;
//        int tX=0;
//        int tY=0;
//        
//        for (PVector point : contours.get(i).getPolygonApproximation ().getPoints()) {
//            points++;
//            xvals+=point.x;
//            yvals+=point.y;
//        }
//     
//        avgX=xvals/points;
//        avgY=yvals/points;
//        float headDepth=pointDepthRaw.get(avgX,avgY)/16777215;
//          if(headDepth<0){
//            headDepth=headDepth+256;
//          }
//        float roughHeadAbs=maxD*(headDepth/256);
//        maxD=roughHeadAbs+.1;
//      }
//    }
//  }
//  kinect.setLowThresholdPC(minD);
//  kinect.setHighThresholdPC(maxD);
//  println("maxD", maxD);
//  
//  
//    contours = opencv.findContours(false, false);
//    if (contours.size() > 0) {
//      int largest_i=0;
//      float largest_size=0;
//      for (int i=0;i<contours.size();i++) {
//        float c_size=contours.get(i).numPoints();
//        if (c_size>largest_size){
//          largest_i=i;
//          largest_size=c_size;
//        }
//      }
//      contours.get(largest_i).setPolygonApproximationFactor(polygonFactor);
//      if (contours.get(largest_i).numPoints() > 250) {
//        //println(contours.get(largest_i).numPoints());
//        int xvals=0;
//        int yvals=0;
//        int points=0;
//        int avgX=0;
//        int avgY=0;
//        int tX=0;
//        int tY=0;
//        int tZ=0;
//        for (PVector point : contours.get(largest_i).getPolygonApproximation ().getPoints()) {
//            points++;
//            xvals+=point.x;
//            yvals+=point.y;
//        }
//        //dynamic clipping
//        avgX=xvals/points;
//        avgY=yvals/points;
//        xPtsQe.add(avgX);
//        yPtsQe.add(avgY);
//        framesAvgX+=avgX;
//        framesAvgY+=avgY;
//        if (yPtsQe.size()>20){
//          tX=xPtsQe.removeFirst();
//          tY=yPtsQe.removeFirst();
//          framesAvgX-=tX;
//          framesAvgY-=tY;
//        }
//        headY=framesAvgY/yPtsQe.size();
//        panx=framesAvgX/xPtsQe.size()-256;
//        headDepth=pointDepthRaw.get(framesAvgX/xPtsQe.size(),framesAvgY/yPtsQe.size())/16777215;
//        if(headDepth<0){
//          headDepth=headDepth+256;
//        }
//        headZ=int(headDepth);
//        zPtsQe.add(headZ);
//        framesAvgZ+=headZ;
//        if (zPtsQe.size()>20){
//          tZ=zPtsQe.removeFirst();
//          framesAvgZ-=tZ;
//        }
//        panZ=(framesAvgZ/zPtsQe.size())-127;
//        }
//        else{
//          maxD=1.25;}
//    }
//    kinect.setLowThresholdPC(minD);
//    kinect.setHighThresholdPC(maxD);
//  }
//}

