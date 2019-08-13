#include "ofApp.h"
#include <ST/CaptureSession.h>

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetFrameRate(60);
	ofBackground(255);
	ofSetColor(255);
	ofEnableDepthTest();
	ofSetVerticalSync(false);

	cam.move(0, .24, 0);
	cam.setNearClip(.35);
	cam.setFarClip(10);

	depth_cam.setParent(cam, true);
	depth_cam.setPosition(cam.getX(), cam.getY(), cam.getZ());
	depth_cam.setNearClip(.58);
	depth_cam.setFarClip(8);

	w = 1280;
	h = 960;

	receiver.startThread();
	st.startThread();

	fbo.allocate(ofGetWidth(), ofGetHeight());

	flipped = false;

	controller.setScale(.010);
	ezcam.setScale(.010);

	float diag_fov = 70.0;

	image_diag = sqrt(pow(w, 2) + pow(h, 2));
	ofLog() << "Image Diagonal: " << image_diag;
	vert_fov = diag_fov * h / image_diag;
	ofLog() << "Vertical FOV: " << vert_fov;
	horiz_fov = diag_fov * w / image_diag;
	ofLog() << "Horizontal FOV: " << horiz_fov;
	focus_len = (h / 2.0) / tan(vert_fov *3.1415/180.0 / 2.0);
	ofLog() << "Focus Length: " << focus_len;
	
	for (int y = 0; y < h; ++y) {
		for (int x = 0; x < w; ++x) {
			abs_height[x + w * y] = abs(h / 2.0 - y);
			abs_width[x + w * y] = abs(w / 2.0 - x);
			pixel_base[x + w * y] = sqrt(pow(focus_len, 2) + pow(abs_width[x + w * y], 2));
			pixel_angle[x + w * y] = atan2(abs_height[x + w * y], pixel_base[x + w * y]);
			pixel_focus[x + w * y] = sqrt(pow(pixel_base[x + w * y], 2) + pow(abs_height[x + w * y], 2));
			pixel_base_ang[x + w * y] = asin(focus_len/pixel_base[x + w * y]);
		}
	}

}
//--------------------------------------------------------------
void ofApp::update(){

	std::stringstream strm;
	strm << "fps: " << ofGetFrameRate();
	ofSetWindowTitle(strm.str());

	// Get the position of the Tracker
	Orientation7 cor = receiver.getCamera();

	ezcam.setOrientation(cor.quat);
	ezcam.setPosition(cor.pos);

	Orientation7 control = receiver.getController();
	controller.setOrientation(control.quat);
	controller.setPosition(control.pos);

	if (control.trigger > 0 && !pressed) {
		pressed = true;
		curve_count = 1;
		positions[0] = control.pos;
		controller.setPosition(control.pos);
		circles.setMatrix(circlenum, controller.getLocalTransformMatrix());
		circles.setColor(circlenum, ofColor::fromHsb(255*control.trigger, 255, 255));
		circles.updateGpu();
		circlenum += 1;
	}
	else if (curve_count < 3 && control.trigger > 0) {
		positions[curve_count]  = control.pos;
		curve_count += 1;
	}
	else if (control.trigger > 0) {
		if (curve_count == 3) {
			positions[3] = control.pos;
			curve_count += 1;
		}
		else {
			positions[0] = positions[1];
			positions[1] = positions[2];
			positions[2] = positions[3];
			positions[3] = control.pos;
		}
		for (int i = 1; i < 11; i++) {
			controller.setPosition(ofInterpolateCatmullRom(positions[0], positions[1], positions[2], positions[3], i * .1));
			circles.setMatrix(circlenum, controller.getGlobalTransformMatrix());
			circles.setColor(circlenum, ofColor::fromHsb(255 * control.trigger, 255, 255));
			circles.updateGpu();
			circlenum += 1;
		}
	}
	else if (control.trigger == 0 && pressed) {
		pressed = false;
	}


	if(st.lastDepthFrame().isValid()){
		std::fill(depth, depth+w*h, 0);
		memcpy(depth, st.lastDepthFrame().depthInMillimeters(), sizeof(float)*w*h);
		
		index = 0;
		for (int y=0; y<h; ++y) {
			for (int x=0; x<w; ++x) {
				if (depth[x + w * y] != 0 && depth[x + w * y] < 8000.0 && depth[x + w * y] == depth[x + w * y]) {
					float* point = points[(x + w * y)].getPtr();

					float obj_height = (depth[x + w * y]/1000.0 + pixel_focus[x + w * y]) *  abs_height[x + w * y] / pixel_focus[x + w * y];
					float actual_pixel_base = sqrt(pow(depth[x + w * y]/1000.0 + pixel_focus[x + w * y],2) - pow(obj_height,2)) - pixel_base[x + w * y];

					points[(x + w * y)] = ezcam.getPosition();
					if (x >= w / 2.0){
						float x_val = (cos(pixel_base_ang[x + w * y]) * actual_pixel_base + abs_width[x + w * y]);
						points[(x + w * y)] += x_val * ezcam.getXAxis() / 1000.0;
					}
						//points[(x + w * y)] += (cos(pixel_base_ang[x + w * y]) * actual_pixel_base + abs_width[x + w * y]) * ezcam.getXAxis() / 1000.0;
						//*point =  ezcam.getX() + (cos(pixel_base_ang[x + w * y]) * actual_pixel_base + abs_width[x + w * y]) / 1000.0;
					else
						points[(x + w * y)] += -(cos(pixel_base_ang[x + w * y]) * actual_pixel_base + abs_width[x + w * y]) * ezcam.getXAxis() / 1000.0;
						//*point = ezcam.getX() - (cos(pixel_base_ang[x + w * y]) * actual_pixel_base + abs_width[x + w * y]) / 1000.0;

					if (y >= h / 2.0)
						points[(x + w * y)] += -obj_height * ezcam.getYAxis() / 1000.0;
						//*(point+1) = ezcam.getY() +  obj_height / 1000.0;
					else
						points[(x + w * y)] += obj_height * ezcam.getYAxis() / 1000.0;
						//*(point+1) = ezcam.getY() - obj_height  / 1000.0;
					points[(x + w * y)] += -sin(pixel_base_ang[x + w * y]) * actual_pixel_base * ezcam.getZAxis();
					//*(point+2) = ezcam.getZ() + sin(pixel_base_ang[x + w * y]) * actual_pixel_base;

					
					faces[index] = x + w * y;
					index += 1;
					
				}
			}
		}
		vbo.setVertexData(&points[0], w*h, GL_DYNAMIC_DRAW);
		vbo.setIndexData(&faces[0], index, GL_DYNAMIC_DRAW);
	}

	depth_cam.move(0, -.24, 0.06);

	fbo.begin();
	ofClear(0,0,0,255);
	ofSetColor(255);

	ezcam.setFov(85); 
	controller.setFov(85);
	ezcam.begin();
	//vbo.drawElements(GL_POINTS, index);
	ezcam.end();

	if (flipped) {
		controller.begin();
		//ezcam.draw();
		vbo.drawElements(GL_POINTS, index);
		controller.end();
	}
	else{
		ezcam.begin();
		vbo.drawElements(GL_POINTS, index);
		ofSetColor(255, 0, 255);
		controller.draw();
		ezcam.end();
	}

	fbo.end();       
}
//--------------------------------------------------------------
void ofApp::draw(){
	ofSetColor(255);
	ofClear(0,0,0,255);
	fbo.draw(0,0);
}

//--------------------------------------------------------------
void ofApp::exit() {
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	switch (key) {
		case 'f':
			flipped = !flipped;
			break;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
