import gab.opencv.*;
import KinectPV2.*;
import java.util.LinkedList;


KinectPV2 kinect;
OpenCV opencv;

float polygonFactor = 1;

int threshold = 10;

float maxD = 1.26f;
float minD = 0.5f;
int framesAvgX=0;
int framesAvgY=0;


boolean contourBodyIndex = false;

LinkedList<Integer> xPtsQe=new LinkedList<Integer>();
LinkedList<Integer> yPtsQe=new LinkedList<Integer>();

boolean running= true;

void setup() { 
  size(512*3, 424, P3D);
  opencv = new OpenCV(this, 512, 424);
  kinect = new KinectPV2(this);
  kinect.enablePointCloud(true);
  kinect.activateRawDepth(true);
  kinect.activateRawColor(true);
  kinect.enableBodyTrackImg(true);
  
  frameRate(120);

  kinect.init();
}

void draw() {
  //thread("posFinder");
//  int [] colorRaw = kinect.getRawColor();
//  int [] depthRaw = kinect.getRawDepth();
//  PImage pointDepthRaw = kinect.getPointCloudDepthImage();
//
//  
  background(0);
//
  noFill();
  strokeWeight(3);
//
  image(kinect.getDepthImage(), 0, 0);
//  //change contour extraction from bodyIndexImg or to PointCloudDepth
//
//  if (contourBodyIndex) {
//    opencv.loadImage(kinect.getBodyTrackImage());
//    opencv.gray();
//    opencv.threshold(threshold);
//    PImage dst = opencv.getOutput();
//  } else {
//    opencv.loadImage(kinect.getPointCloudDepthImage());
//    opencv.gray();
//    opencv.threshold(threshold);
//    PImage dst = opencv.getOutput();
//  }
//  maxD=1.26;
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
//  image(pointDepthRaw, 512, 0);
//  contours = opencv.findContours(false, false);
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
//        stroke(0, 200, 200);
//        beginShape();
//
//        for (PVector point : contours.get(i).getPolygonApproximation ().getPoints()) {
//          vertex(point.x + 512*2, point.y);
//            points++;
//            xvals+=point.x;
//            yvals+=point.y;
//        }
//        endShape();    
//        //dynamic clipping
//        //integrate with video
//        //zoom/perspective
//        //spawn multiple threads
//        avgX=xvals/points;
//        avgY=yvals/points;
//        xPtsQe.add(avgX);
//        yPtsQe.add(avgY);
//        framesAvgX+=avgX;
//        framesAvgY+=avgY;
//        if (yPtsQe.size()>15){
//          tX=xPtsQe.removeFirst();
//          tY=yPtsQe.removeFirst();
//          framesAvgX-=tX;
//          framesAvgY-=tY;
//        }
//        //println(framesAvgX/xPtsQe.size()+ 512*2,framesAvgY/yPtsQe.size());
//        point(framesAvgX/xPtsQe.size()+ 512,framesAvgY/yPtsQe.size());
//        //println(pointDepthRaw.get(floor(xvals/points)/*512*/, floor(yvals/points))/10000);
//        int headDepth=pointDepthRaw.get(framesAvgX/xPtsQe.size(),framesAvgY/yPtsQe.size())/16777215;
//          if(headDepth<0){
//            headDepth=headDepth+256;
//          }
//          println(headDepth);
//          
//      }
//    }
//  }
//
  noStroke();
  fill(0);
  rect(0, 0, 130, 100);
  fill(255, 0, 0);
  text("fps: "+frameRate, 20, 20);
  text("threshold: "+threshold, 20, 40);
  text("minD: "+minD, 20, 60);
  text("maxD: "+maxD, 20, 80);
}
void keyPressed() {
  //change contour finder from contour body to depth-PC
  if( key == 'b'){
   contourBodyIndex = !contourBodyIndex;
   if(contourBodyIndex)
     threshold = 200;
    else
     threshold = 40;
  }
  
  if (key == 'a') {
    threshold+=1;
  }
  if (key == 's') {
    threshold-=1;
  }

  if (key == '1') {
    minD += 0.01;
  }

  if (key == '2') {
    minD -= 0.01;
  }

  if (key == '3') {
    maxD += 0.01;
  }

  if (key == '4') {
    maxD -= 0.01;
  }

  if (key == '5')
    polygonFactor += 0.1;

  if (key == '6')
    polygonFactor -= 0.1;
}
void stop() {
  running = false;
  super.stop();
}

void posFinder(){
  while (running) {
      int [] colorRaw = kinect.getRawColor();
  int [] depthRaw = kinect.getRawDepth();
  PImage pointDepthRaw = kinect.getPointCloudDepthImage();

//  
//
//
//  //change contour extraction from bodyIndexImg or to PointCloudDepth
//
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
  image(pointDepthRaw, 512, 0);
  contours = opencv.findContours(false, false);
  
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
        
        stroke(0, 200, 200);
        beginShape();

        for (PVector point : contours.get(i).getPolygonApproximation ().getPoints()) {
          vertex(point.x + 512*2, point.y);
            points++;
            xvals+=point.x;
            yvals+=point.y;
        }
        endShape();    
        //dynamic clipping
        //integrate with video
        //zoom/perspective
        //spawn multiple threads
        avgX=xvals/points;
        avgY=yvals/points;
        xPtsQe.add(avgX);
        yPtsQe.add(avgY);
        framesAvgX+=avgX;
        framesAvgY+=avgY;
        if (yPtsQe.size()>15){
          tX=xPtsQe.removeFirst();
          tY=yPtsQe.removeFirst();
          framesAvgX-=tX;
          framesAvgY-=tY;
        }
        //println(framesAvgX/xPtsQe.size()+ 512*2,framesAvgY/yPtsQe.size());
        point(framesAvgX/xPtsQe.size()+ 512,framesAvgY/yPtsQe.size());
        //println(pointDepthRaw.get(floor(xvals/points)/*512*/, floor(yvals/points))/10000);
        int headDepth=pointDepthRaw.get(framesAvgX/xPtsQe.size(),framesAvgY/yPtsQe.size())/16777215;
          if(headDepth<0){
            headDepth=headDepth+256;
          }
          println(headDepth);
          
      }
    }
  }

  }
}

