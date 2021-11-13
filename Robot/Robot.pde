void setup(){
  size(640,480);
  surface.setTitle("Inverse Kinematics Scenario");
}

float armW = 20;
float skeletonWidth = 80;
float skeletonHeight = 100;
float radius = 50;

//obstacles
int numObstacles = 0;
static int maxNumObstacles = 50;
Vec2 circlePos[] = new Vec2[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii
float configureRad[] =  new float[maxNumObstacles]; //configuration space for every obstacle

//Root
Vec2 lroot = new Vec2(280,200);
Vec2 rroot = new Vec2(lroot.x+skeletonWidth,lroot.y);

//Object
Vec2 objPos = new Vec2(200,380);
int objW = 30;
int objH = 30;

//Upper Arm
float l0 = 80; 
float la0 = 1.57; //Shoulder joint
float ra0 = 1.57;


//Lower Arm
float l1 = 60;
float la1 = -0.3; //Elbow joint
float ra1 = 0.3;


//Hand
float l2 = 30;
float la2 = -0.3; //Wrist joint
float ra2 = 0.3;


//finger
float l3 = 20;
float la3 = -0.3; //finger joint
float ra3 = 0.3;


Vec2 lstart_l1,lstart_l2,lstart_l3,lendPoint;
Vec2 rstart_l1,rstart_l2,rstart_l3,rendPoint;

void update(){
  Vec2 lgoal = new Vec2(objPos.x, objPos.y);
  Vec2 rgoal = new Vec2(objPos.x+objW, objPos.y);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //update the left side
  //Update finger joint
  startToGoal = lgoal.minus(lstart_l3);
  startToEndEffector = lendPoint.minus(lstart_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  
  if (cross(startToGoal,startToEndEffector) < 0){
    if((la3+angleDiff) >= 1.57) 
        la3 = 1.57;
    else 
        //la3 += angleDiff;
        la3 = lerp(la3, la3+angleDiff, 0.2);
     
  }
  else {
    if((la3-angleDiff) <= -1.57) 
        la3 = -1.57;
    else
        //la3 -= angleDiff;
        la3 = lerp(la3, la3-angleDiff, 0.2);
    
  }
    
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  //Update wrist joint
  startToGoal = lgoal.minus(lstart_l2);
  startToEndEffector = lendPoint.minus(lstart_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    if((la2+angleDiff) >= 1.57) 
        la2 = 1.57;
    else {
        //la2 += angleDiff;
        la2 = lerp(la2, la2+angleDiff, 0.15);
    }
   
  }
  else {
    if((la2-angleDiff) <= -1.57) 
        la2 = -1.57;
    else
        //la2 -= angleDiff;
        la2 = lerp(la2, la2-angleDiff, 0.15);
        
  }
    
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update elbow joint
  startToGoal = lgoal.minus(lstart_l1);
  startToEndEffector = lendPoint.minus(lstart_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    if((la1+angleDiff) >= 1.57) 
        la1 = 1.57;
    else
        //la1 += angleDiff;
        la1 = lerp(la1, la1+angleDiff, 0.1);
        
    
  }
  else {
    if((la1-angleDiff) <= -1.57) 
        la1 = -1.57;
    else
        //la1 -= angleDiff;
        la1 = lerp(la1, la1-angleDiff, 0.1);
         
        
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update shoulder joint
  startToGoal = lgoal.minus(lroot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = lendPoint.minus(lroot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0) {
   /*if((la0+angleDiff) >= 1.57) 
        la0 = 1.57;
   else*/
        //la0 += angleDiff;
        la0 = lerp(la0, la0+angleDiff, 0.08);
        
        
  }
  else {
    /*if((la0-angleDiff) <= 0) 
        la0 = 0;
   else*/
        //la0 -= angleDiff;
        la0 = lerp(la0, la0-angleDiff, 0.08);
       
  }
   
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  //update the right side
  //Update finger joint
  startToGoal = rgoal.minus(rstart_l3);
  startToEndEffector = rendPoint.minus(rstart_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    if((ra3+angleDiff) >= 1.57) 
        ra3 = 1.57;
    else
        //ra3 += angleDiff;
        ra3 = lerp(ra3, ra3+angleDiff, 0.2);
        //ra3 += angleDiff;
  }
  else {
    if((ra3-angleDiff) <= -1.57) 
        ra3 = -1.57;
    else
        //ra3 -= angleDiff;
        ra3 = lerp(ra3, ra3-angleDiff, 0.2);
        //ra3 -= angleDiff;
        
  }
    
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  //Update wrist joint
  startToGoal = rgoal.minus(rstart_l2);
  startToEndEffector = rendPoint.minus(rstart_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    if((ra2+angleDiff) >= 1.57) 
        ra2 = 1.57;
    else
        //ra2 += angleDiff;
        ra2 = lerp(ra2, ra2+angleDiff, 0.15);
         //ra2 += angleDiff;
  }
  else {
    if((ra2-angleDiff) <= -1.57) 
        ra2 = -1.57;
    else
        //ra2 -= angleDiff;
        ra2 = lerp(ra2, ra2-angleDiff, 0.15);
        //ra2 -= angleDiff;
  }
    
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update elbow joint
  startToGoal = rgoal.minus(rstart_l1);
  startToEndEffector = rendPoint.minus(rstart_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0){
    if((ra1+angleDiff) >= 1.57) 
        ra1 = 1.57;
    else
        //ra1 += angleDiff;
        ra1 = lerp(ra1, ra1+angleDiff, 0.1);
        //ra1 += angleDiff;
    
  }
  else {
    if((ra1-angleDiff) <= -1.57) 
        ra1 = -1.57;
    else
        //ra1 -= angleDiff;
        ra1 = lerp(ra1, ra1-angleDiff, 0.1);
         //ra1 -= angleDiff;
  }
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  
  //Update shoulder joint
  startToGoal = rgoal.minus(rroot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = rendPoint.minus(rroot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  if (cross(startToGoal,startToEndEffector) < 0) {
   /*if((la0+angleDiff) >= 1.57) 
        la0 = 1.57;
   else*/
        //ra0 += angleDiff;
      ra0 = lerp(ra0, ra0+angleDiff, 0.08);
        //ra0 += angleDiff;
  }
  else {
    /*if((la0-angleDiff) <= 0) 
        la0 = 0;
   else*/
      ra0 = lerp(ra0, ra0-angleDiff, 0.08);
        //ra0 -= angleDiff;
        //ra0 -= angleDiff;
  }
   
  fk(); //Update link positions with fk (e.g. end effector changed)
 
  //println("Angle 0:",la0,"Angle 1:",la1,"Angle 2:",la2,"Angle 2:",la3);
}

void fk(){ 
  //collsion detect and fix
  hitInfo hit = new hitInfo();
  float minT = hit.t;
  //left arm
  lstart_l1 = new Vec2(cos(la0)*l0,sin(la0)*l0).plus(lroot);
  for(int i = 0; i < numObstacles; i++) {
    Vec2 center = circlePos[i];
    float r = circleRad[i]+armW/2;
    Vec2 l_dir = lstart_l1.minus(lroot);
    float l_len = l_dir.length(); 
    l_dir.normalize();
    hitInfo circleHit = lineCircleIntesect(center, r, lroot, l_dir, l_len, hit.t);
    hit.hit = circleHit.hit;
    hit.t = circleHit.t;
    if(hit.hit == true && hit.t == -1) {
      Vec2 dir = lroot.minus(center);
      dir.normalize();
      lroot = center.plus(dir.times(r+0.01));
      float templa0 = la0;
      Vec2  tempPos = lstart_l1;
      if(la0 < 0) {
        templa0 -= 0.01;
        tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(lroot);
        l_dir = tempPos.minus(lroot);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lroot, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa0 -= 0.01;
             tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(lroot);
             l_dir = tempPos.minus(lroot);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la0 = templa0;
             lstart_l1 = new Vec2(cos(la0)*l0,sin(la0)*l0).plus(lroot);
             break;
           }
        }
     }
     else if(la0 >= 0) {
        templa0 += 0.01;
        tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(lroot);
        l_dir = tempPos.minus(lroot);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lroot, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa0 += 0.01;
             tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(lroot);
             l_dir = tempPos.minus(lroot);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la0 = templa0;
             lstart_l1 = new Vec2(cos(la0)*l0,sin(la0)*l0).plus(lroot);
             break;
           }
        }
     }
   }
    if(hit.hit == true) {
      if(hit.t < minT && hit.t != -1) {
        float templa0 = la0;
        Vec2  tempPos = lstart_l1;
         if(la0 < 0) {
           templa0 -= 0.01;
           tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(lroot);
           l_dir = tempPos.minus(lroot);
           l_len =  l_dir.length(); 
           l_dir.normalize();
           while(true) {
             circleHit = lineCircleIntesect(center, r, lroot, l_dir, l_len, hit.t);
             if (circleHit.hit) {
               templa0 -= 0.01;
               tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(lroot);
               l_dir = tempPos.minus(lroot);
               l_len =  l_dir.length(); 
               l_dir.normalize();
             }
             else{
               la0 = templa0;
               lstart_l1 = new Vec2(cos(la0)*l0,sin(la0)*l0).plus(lroot);
               break;
             }
           }
         } 
         else if(la0 >= 0) {
           templa0 += 0.01;
           tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(lroot);
           l_dir = tempPos.minus(lroot);
           l_len =  l_dir.length(); 
           l_dir.normalize();
           while(true) {
             circleHit = lineCircleIntesect(center, r, lroot, l_dir, l_len, hit.t);
             if (circleHit.hit) {
               templa0 += 0.01;
               tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(lroot);
               l_dir = tempPos.minus(lroot);
               l_len =  l_dir.length(); 
               l_dir.normalize();
             }
             else{
               la0 = templa0;
               lstart_l1 = new Vec2(cos(la0)*l0,sin(la0)*l0).plus(lroot);
               break;
             }
           }
         }
     }
   }
 }
  
 
  lstart_l2 = new Vec2(cos(la0+la1)*l1,sin(la0+la1)*l1).plus(lstart_l1);
  for(int i = 0; i < numObstacles; i++) {
    Vec2 center = circlePos[i];
    float r = circleRad[i]+armW/2;
    Vec2 l_dir = lstart_l2.minus(lstart_l1);
    float l_len = l_dir.length(); 
    l_dir.normalize();
    hitInfo circleHit = lineCircleIntesect(center, r, lstart_l1, l_dir, l_len, hit.t);
    hit.hit = circleHit.hit;
    hit.t = circleHit.t;
    if(hit.hit == true && hit.t == -1) {
      Vec2 dir = lstart_l1.minus(center);
      dir.normalize();
      lstart_l1 = center.plus(dir.times(r+0.01));
      float templa1 = la1;
      Vec2  tempPos = lstart_l2;
      if(la1 < 0) {
        templa1 -= 0.01;
        tempPos = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
        l_dir = tempPos.minus(lstart_l1);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l1, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa1 -= 0.01;
             tempPos = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
             l_dir = tempPos.minus(lstart_l1);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la1 = templa1;
             lstart_l2 = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
             break;
           }
        }
     }
     else if(la0 >= 0) {
        templa1 += 0.01;
        tempPos = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
        l_dir = tempPos.minus(lstart_l1);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l1, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa1 += 0.01;
             tempPos = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
             l_dir = tempPos.minus(lstart_l1);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la1 = templa1;
             lstart_l2 = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
             break;
           }
        }
     }
   }
    if(hit.hit == true) {
      if(hit.t < minT && hit.t != -1) {
        float templa1 = la1;
        Vec2  tempPos = lstart_l2;
         if(la1 < 0) {
           templa1 -= 0.01;
           tempPos = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
           l_dir = tempPos.minus(lstart_l1);
           l_len =  l_dir.length(); 
           l_dir.normalize();
           while(true) {
             circleHit = lineCircleIntesect(center, r, lstart_l1, l_dir, l_len, hit.t);
             if (circleHit.hit) {
               templa1 -= 0.01;
               tempPos = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
               l_dir = tempPos.minus(lstart_l1);
               l_len =  l_dir.length(); 
               l_dir.normalize();
             }
             else{
               la1 = templa1;
               lstart_l2 = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
               break;
             }
          }
        }
        else if(la1 >= 0) {
          templa1 += 0.01;
          tempPos = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
          l_dir = tempPos.minus(lstart_l1);
          l_len =  l_dir.length(); 
          l_dir.normalize();
          while(true) {
             circleHit = lineCircleIntesect(center, r, lstart_l1, l_dir, l_len, hit.t);
             if (circleHit.hit) {
               templa1 += 0.01;
               tempPos = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
               l_dir = tempPos.minus(lstart_l1);
               l_len =  l_dir.length(); 
               l_dir.normalize();
             }
             else{
               la1 = templa1;
               lstart_l2 = new Vec2(cos(la0+templa1)*l1,sin(la0+templa1)*l1).plus(lstart_l1);
               break;
             }
          }
       }
     }
   }
  }
  lstart_l3 = new Vec2(cos(la0+la1+la2)*l2,sin(la0+la1+la2)*l2).plus(lstart_l2);
  for(int i = 0; i < numObstacles; i++) {
    Vec2 center = circlePos[i];
    float r = circleRad[i]+armW/2;
    Vec2 l_dir = lstart_l3.minus(lstart_l2);
    float l_len = l_dir.length(); 
    l_dir.normalize();
    hitInfo circleHit = lineCircleIntesect(center, r, lstart_l2, l_dir, l_len, hit.t);
    hit.hit = circleHit.hit;
    hit.t = circleHit.t;
    if(hit.hit == true && hit.t == -1) {
      Vec2 dir = lstart_l2.minus(center);
      dir.normalize();
      lstart_l2 = center.plus(dir.times(r+0.01));
      float templa2 = la2;
      Vec2  tempPos = lstart_l3;
      if(la2 < 0) {
        templa2 -= 0.01;
        tempPos = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
        l_dir = tempPos.minus(lstart_l2);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l2, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa2 -= 0.01;
             tempPos = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
             l_dir = tempPos.minus(lstart_l2);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la2 = templa2;
             lstart_l3 = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
             break;
           }
        }
     }
     else if(la2 >= 0) {
        templa2 += 0.01;
        tempPos = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
        l_dir = tempPos.minus(lstart_l2);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l2, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa2 += 0.01;
             tempPos = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
             l_dir = tempPos.minus(lstart_l2);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la2 = templa2;
             lstart_l3 = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
             break;
           }
        }
     }
   }
    if(hit.hit == true) {
      if(hit.t < minT && hit.t != -1) {
        float templa2 = la2;
      Vec2  tempPos = lstart_l3;
         
      if(la2 < 0) {
        templa2 -= 0.01;
        tempPos = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
        l_dir = tempPos.minus(lstart_l2);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l2, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa2 -= 0.01;
             tempPos = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
             l_dir = tempPos.minus(lstart_l2);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la2 = templa2;
             lstart_l3 = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
             break;
           }
        }
     }
     else if(la2 >= 0) {
        templa2 += 0.01;
        tempPos = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
        l_dir = tempPos.minus(lstart_l2);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l2, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa2 += 0.01;
             tempPos = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
             l_dir = tempPos.minus(lstart_l2);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la2 = templa2;
             lstart_l3 = new Vec2(cos(la0+la1+templa2)*l2,sin(la0+la1+templa2)*l2).plus(lstart_l2);
             break;
           }
        }
     }
    }
   }
  }
  
  
  lendPoint = new Vec2(cos(la0+la1+la2+la3)*l3,sin(la0+la1+la2+la3)*l3).plus(lstart_l3);
  for(int i = 0; i < numObstacles; i++) {
    Vec2 center = circlePos[i];
    float r = circleRad[i]+armW/2;
    Vec2 l_dir = lendPoint.minus(lstart_l3);
    float l_len = l_dir.length(); 
    l_dir.normalize();
    hitInfo circleHit = lineCircleIntesect(center, r, lstart_l3, l_dir, l_len, hit.t);
    hit.hit = circleHit.hit;
    hit.t = circleHit.t;
    if(hit.hit == true && hit.t == -1) {
      Vec2 dir = lstart_l3.minus(center);
      dir.normalize();
      lstart_l3 = center.plus(dir.times(r+0.01));
      float templa3 = la3;
      Vec2  tempPos = lendPoint;
      if(la3 < 0) {
        templa3 -= 0.01;
        tempPos = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
        l_dir = tempPos.minus(lstart_l3);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l3, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa3 -= 0.01;
             tempPos = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
             l_dir = tempPos.minus(lstart_l3);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la3 = templa3;
             lendPoint = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
             break;
           }
        }
     }
     else if(la3 >= 0) {
        templa3 += 0.01;
        tempPos = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
        l_dir = tempPos.minus(lstart_l3);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l3, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa3 += 0.01;
             tempPos = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
             l_dir = tempPos.minus(lstart_l3);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la3 = templa3;
             lendPoint = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
             break;
           }
        }
     }
   }
    if(hit.hit == true) {
      if(hit.t < minT && hit.t != -1) {
        float templa3 = la3;
      Vec2  tempPos = lendPoint;
         
      if(la3 < 0) {
        templa3 -= 0.01;
        tempPos = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
        l_dir = tempPos.minus(lstart_l3);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l3, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa3 -= 0.01;
             tempPos = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
             l_dir = tempPos.minus(lstart_l3);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la3 = templa3;
             lendPoint = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
             break;
           }
        }
     }
     else if(la3 >= 0) {
        templa3 += 0.01;
        tempPos = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
        l_dir = tempPos.minus(lstart_l3);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, lstart_l3, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa3 += 0.01;
             tempPos = new Vec2(cos(la0+la1+la2+templa3)*l2,sin(la0+la1+la2+templa3)*l2).plus(lstart_l3);
             l_dir = tempPos.minus(lstart_l3);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             la3 = templa3;
             lendPoint = new Vec2(cos(la0+la1+la2+templa3)*l3,sin(la0+la1+la2+templa3)*l3).plus(lstart_l3);
             break;
           }
        }
     }
    }
   }
  }
  
  //right arm
  rroot = new Vec2(lroot.x+skeletonWidth, lroot.y);
  rstart_l1 = new Vec2(cos(ra0)*l0,sin(ra0)*l0).plus(rroot);
  for(int i = 0; i < numObstacles; i++) {
    Vec2 center = circlePos[i];
    float r = circleRad[i]+armW/2;
    Vec2 l_dir = rstart_l1.minus(rroot);
    float l_len = l_dir.length(); 
    l_dir.normalize();
    hitInfo circleHit = lineCircleIntesect(center, r, rroot, l_dir, l_len, hit.t);
    hit.hit = circleHit.hit;
    hit.t = circleHit.t;
    if(hit.hit == true && hit.t == -1) {
      Vec2 dir = rroot.minus(center);
      dir.normalize();
      rroot = center.plus(dir.times(r+0.01));
      float templa0 = ra0;
      Vec2  tempPos = rstart_l1;
      if(ra0 < 0) {
        templa0 -= 0.01;
        tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(rroot);
        l_dir = tempPos.minus(rroot);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rroot, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa0 -= 0.01;
             tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(rroot);
             l_dir = tempPos.minus(rroot);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra0 = templa0;
             rstart_l1 = new Vec2(cos(ra0)*l0,sin(ra0)*l0).plus(rroot);
             break;
           }
        }
     }
     else if(ra0 >= 0) {
        templa0 += 0.01;
        tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(rroot);
        l_dir = tempPos.minus(rroot);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rroot, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa0 += 0.01;
             tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(rroot);
             l_dir = tempPos.minus(rroot);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra0 = templa0;
             rstart_l1 = new Vec2(cos(ra0)*l0,sin(ra0)*l0).plus(rroot);
             break;
           }
        }
     }
   }
    if(hit.hit == true) {
      if(hit.t < minT && hit.t != -1) {
        float templa0 = ra0;
        Vec2  tempPos = rstart_l1;
         if(ra0 < 0) {
           templa0 -= 0.01;
           tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(rroot);
           l_dir = tempPos.minus(rroot);
           l_len =  l_dir.length(); 
           l_dir.normalize();
           while(true) {
             circleHit = lineCircleIntesect(center, r, rroot, l_dir, l_len, hit.t);
             if (circleHit.hit) {
               templa0 -= 0.01;
               tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(rroot);
               l_dir = tempPos.minus(rroot);
               l_len =  l_dir.length(); 
               l_dir.normalize();
             }
             else{
               ra0 = templa0;
               rstart_l1 = new Vec2(cos(ra0)*l0,sin(ra0)*l0).plus(rroot);
               break;
             }
           }
         } 
         else if(ra0 >= 0) {
           templa0 += 0.01;
           tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(rroot);
           l_dir = tempPos.minus(rroot);
           l_len =  l_dir.length(); 
           l_dir.normalize();
           while(true) {
             circleHit = lineCircleIntesect(center, r, rroot, l_dir, l_len, hit.t);
             if (circleHit.hit) {
               templa0 += 0.01;
               tempPos = new Vec2(cos(templa0)*l0,sin(templa0)*l0).plus(rroot);
               l_dir = tempPos.minus(rroot);
               l_len =  l_dir.length(); 
               l_dir.normalize();
             }
             else{
               ra0 = templa0;
               rstart_l1 = new Vec2(cos(ra0)*l0,sin(ra0)*l0).plus(rroot);
               break;
             }
           }
         }
     }
   }
 }
 
  rstart_l2 = new Vec2(cos(ra0+ra1)*l1,sin(ra0+ra1)*l1).plus(rstart_l1);
  for(int i = 0; i < numObstacles; i++) {
    Vec2 center = circlePos[i];
    float r = circleRad[i]+armW/2;
    Vec2 l_dir = rstart_l2.minus(rstart_l1);
    float l_len = l_dir.length(); 
    l_dir.normalize();
    hitInfo circleHit = lineCircleIntesect(center, r, rstart_l1, l_dir, l_len, hit.t);
    hit.hit = circleHit.hit;
    hit.t = circleHit.t;
    if(hit.hit == true && hit.t == -1) {
      Vec2 dir = rstart_l1.minus(center);
      dir.normalize();
      rstart_l1 = center.plus(dir.times(r+0.01));
      float templa1 = ra1;
      Vec2  tempPos = rstart_l2;
      if(ra1 < 0) {
        templa1 -= 0.01;
        tempPos = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
        l_dir = tempPos.minus(rstart_l1);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l1, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa1 -= 0.01;
             tempPos = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
             l_dir = tempPos.minus(rstart_l1);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra1 = templa1;
             rstart_l2 = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
             break;
           }
        }
     }
     else if(ra0 >= 0) {
        templa1 += 0.01;
        tempPos = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
        l_dir = tempPos.minus(rstart_l1);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l1, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa1 += 0.01;
             tempPos = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
             l_dir = tempPos.minus(rstart_l1);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra1 = templa1;
             rstart_l2 = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
             break;
           }
        }
     }
   }
    if(hit.hit == true) {
      if(hit.t < minT && hit.t != -1) {
        float templa1 = ra1;
        Vec2  tempPos = rstart_l2;
         if(ra1 < 0) {
           templa1 -= 0.01;
           tempPos = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
           l_dir = tempPos.minus(rstart_l1);
           l_len =  l_dir.length(); 
           l_dir.normalize();
           while(true) {
             circleHit = lineCircleIntesect(center, r, rstart_l1, l_dir, l_len, hit.t);
             if (circleHit.hit) {
               templa1 -= 0.01;
               tempPos = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
               l_dir = tempPos.minus(rstart_l1);
               l_len =  l_dir.length(); 
               l_dir.normalize();
             }
             else{
               ra1 = templa1;
               rstart_l2 = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
               break;
             }
          }
        }
        else if(ra1 >= 0) {
          templa1 += 0.01;
          tempPos = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
          l_dir = tempPos.minus(rstart_l1);
          l_len =  l_dir.length(); 
          l_dir.normalize();
          while(true) {
             circleHit = lineCircleIntesect(center, r, rstart_l1, l_dir, l_len, hit.t);
             if (circleHit.hit) {
               templa1 += 0.01;
               tempPos = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
               l_dir = tempPos.minus(rstart_l1);
               l_len =  l_dir.length(); 
               l_dir.normalize();
             }
             else{
               ra1 = templa1;
               rstart_l2 = new Vec2(cos(ra0+templa1)*l1,sin(ra0+templa1)*l1).plus(rstart_l1);
               break;
             }
          }
       }
     }
   }
  }
  
  rstart_l3 = new Vec2(cos(ra0+ra1+ra2)*l2,sin(ra0+ra1+ra2)*l2).plus(rstart_l2);
  for(int i = 0; i < numObstacles; i++) {
    Vec2 center = circlePos[i];
    float r = circleRad[i]+armW/2;
    Vec2 l_dir = rstart_l3.minus(rstart_l2);
    float l_len = l_dir.length(); 
    l_dir.normalize();
    hitInfo circleHit = lineCircleIntesect(center, r, rstart_l2, l_dir, l_len, hit.t);
    hit.hit = circleHit.hit;
    hit.t = circleHit.t;
    if(hit.hit == true && hit.t == -1) {
      Vec2 dir = rstart_l2.minus(center);
      dir.normalize();
      rstart_l2 = center.plus(dir.times(r+0.01));
      float templa2 = ra2;
      Vec2  tempPos = rstart_l3;
      if(ra2 < 0) {
        templa2 -= 0.01;
        tempPos = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
        l_dir = tempPos.minus(rstart_l2);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l2, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa2 -= 0.01;
             tempPos = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
             l_dir = tempPos.minus(rstart_l2);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra2 = templa2;
             rstart_l3 = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
             break;
           }
        }
     }
     else if(ra2 >= 0) {
        templa2 += 0.01;
        tempPos = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
        l_dir = tempPos.minus(rstart_l2);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l2, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa2 += 0.01;
             tempPos = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
             l_dir = tempPos.minus(rstart_l2);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra2 = templa2;
             rstart_l3 = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
             break;
           }
        }
     }
   }
    if(hit.hit == true) {
      if(hit.t < minT && hit.t != -1) {
        float templa2 = ra2;
      Vec2  tempPos = rstart_l3;
         
      if(ra2 < 0) {
        templa2 -= 0.01;
        tempPos = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
        l_dir = tempPos.minus(rstart_l2);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l2, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa2 -= 0.01;
             tempPos = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
             l_dir = tempPos.minus(rstart_l2);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra2 = templa2;
             rstart_l3 = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
             break;
           }
        }
     }
     else if(ra2 >= 0) {
        templa2 += 0.01;
        tempPos = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
        l_dir = tempPos.minus(rstart_l2);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l2, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa2 += 0.01;
             tempPos = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
             l_dir = tempPos.minus(rstart_l2);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra2 = templa2;
             rstart_l3 = new Vec2(cos(ra0+ra1+templa2)*l2,sin(ra0+ra1+templa2)*l2).plus(rstart_l2);
             break;
           }
        }
     }
    }
   }
  }
  
  rendPoint = new Vec2(cos(ra0+ra1+ra2+ra3)*l3,sin(ra0+ra1+ra2+ra3)*l3).plus(rstart_l3);
  for(int i = 0; i < numObstacles; i++) {
    Vec2 center = circlePos[i];
    float r = circleRad[i]+armW/2;
    Vec2 l_dir = rendPoint.minus(rstart_l3);
    float l_len = l_dir.length(); 
    l_dir.normalize();
    hitInfo circleHit = lineCircleIntesect(center, r, rstart_l3, l_dir, l_len, hit.t);
    hit.hit = circleHit.hit;
    hit.t = circleHit.t;
    if(hit.hit == true && hit.t == -1) {
      Vec2 dir = rstart_l3.minus(center);
      dir.normalize();
      rstart_l3 = center.plus(dir.times(r+0.01));
      float templa3 = ra3;
      Vec2  tempPos = rendPoint;
      if(ra3 < 0) {
        templa3 -= 0.01;
        tempPos = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
        l_dir = tempPos.minus(rstart_l3);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l3, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa3 -= 0.01;
             tempPos = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
             l_dir = tempPos.minus(rstart_l3);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra3 = templa3;
             rendPoint = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
             break;
           }
        }
     }
     else if(ra3 >= 0) {
        templa3 += 0.01;
        tempPos = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
        l_dir = tempPos.minus(rstart_l3);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l3, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa3 += 0.01;
             tempPos = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
             l_dir = tempPos.minus(rstart_l3);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra3 = templa3;
             rendPoint = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
             break;
           }
        }
     }
   }
    if(hit.hit == true) {
      if(hit.t < minT && hit.t != -1) {
        float templa3 = ra3;
      Vec2  tempPos = rendPoint;
         
      if(ra3 < 0) {
        templa3 -= 0.01;
        tempPos = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
        l_dir = tempPos.minus(rstart_l3);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l3, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa3 -= 0.01;
             tempPos = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
             l_dir = tempPos.minus(rstart_l3);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra3 = templa3;
             rendPoint = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
             break;
           }
        }
     }
     else if(ra3 >= 0) {
        templa3 += 0.01;
        tempPos = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
        l_dir = tempPos.minus(rstart_l3);
        l_len =  l_dir.length(); 
        l_dir.normalize();
        while(true) {
           circleHit = lineCircleIntesect(center, r, rstart_l3, l_dir, l_len, hit.t);
           if (circleHit.hit) {
             templa3 += 0.01;
             tempPos = new Vec2(cos(ra0+ra1+ra2+templa3)*l2,sin(ra0+ra1+ra2+templa3)*l2).plus(rstart_l3);
             l_dir = tempPos.minus(rstart_l3);
             l_len =  l_dir.length(); 
             l_dir.normalize();
           }
           else{
             ra3 = templa3;
             rendPoint = new Vec2(cos(ra0+ra1+ra2+templa3)*l3,sin(ra0+ra1+ra2+templa3)*l3).plus(rstart_l3);
             break;
           }
        }
     }
    }
   }
  }
  
  
  
  
}


void draw(){
  fk();
  if(!paused){
     update();
  }
 
  
  background(250,250,250);
  
  //draw object
  fill(0,0,0);
  pushMatrix();
  translate(objPos.x, objPos.y);
  rect(0, 0, objW,objH);
  popMatrix();
  
 
  
  fill(240,191,153);
  circle(lroot.x+skeletonWidth/2, lroot.y-10, radius);
  
  
  pushMatrix();
  translate(lroot.x,lroot.y);
  rect(0, 0, skeletonWidth, skeletonHeight);
  popMatrix();

  fill(240,191,153);
  pushMatrix();
  translate(lroot.x,lroot.y);
  rotate(la0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(lstart_l1.x,lstart_l1.y);
  rotate(la0+la1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(lstart_l2.x,lstart_l2.y);
  rotate(la0+la1+la2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  
  pushMatrix();
  translate(lstart_l3.x,lstart_l3.y);
  rotate(la0+la1+la2+la3);
  rect(0, -armW/2, l3, armW);
  popMatrix();
  
  pushMatrix();
  translate(lroot.x+skeletonWidth,lroot.y);
  rotate(ra0);
  rect(0, -armW/2, l0, armW);
  popMatrix();
  
  pushMatrix();
  translate(rstart_l1.x,rstart_l1.y);
  rotate(ra0+ra1);
  rect(0, -armW/2, l1, armW);
  popMatrix();
  
  pushMatrix();
  translate(rstart_l2.x,rstart_l2.y);
  rotate(ra0+ra1+ra2);
  rect(0, -armW/2, l2, armW);
  popMatrix();
  
  pushMatrix();
  translate(rstart_l3.x,rstart_l3.y);
  rotate(ra0+ra1+ra2+ra3);
  rect(0, -armW/2, l3, armW);
  popMatrix();
  
  //draw obstacles
  Vec2 mousePos = new Vec2(mouseX, mouseY);
  for (int i = 0; i < numObstacles; i++){
    
    if(pointInCircle(circlePos[i], circleRad[i], mousePos, 1)){
      stroke(255,0,0);
      
    }
     
    Vec2 c = circlePos[i];
    float r = circleRad[i];
    fill(255,255,0);
    circle(c.x,c.y,r*2);
    stroke(0);
    
  }
}

boolean pointInCircle(Vec2 center, float r, Vec2 pointPos, float eps){
  if(eps == 0) {
    float dis = pow(pointPos.x-center.x, 2) + pow(pointPos.y-center.y, 2);
    if(dis > pow(r,2)) {
      return false;
    }
    return true;
  }
  else {
    float dis = pow(pointPos.x-center.x, 2) + pow(pointPos.y-center.y, 2);
    if(dis > pow(r+eps,2)) {
      return false;
    }
    return true;
  }
}
boolean paused = true;
void keyPressed()
{
  if(key == ' ') {  //press space to enable/disable debug mode
     paused = !paused;
  }
}

boolean pointInBox(Vec2 boxTopLeft, float boxW, float boxH, Vec2 pointPos){
  if(pointPos.x < boxTopLeft.x || pointPos.x > boxTopLeft.x + boxW)
     return false;
  if(pointPos.y < boxTopLeft.y || pointPos.y > boxTopLeft.y + boxH)
     return false;
  return true;
}



class hitInfo{
  public boolean hit = false;
  public float t = 9999999;
}

//Find the first point on a circle hit when starting at l_start and traveling in direction l_dir
//NOTE: This will only count collisions less than max_t away from l_start
hitInfo lineCircleIntesect(Vec2 center, float r, Vec2 l_start, Vec2 l_dir, float l_len, float max_t){
  
  hitInfo hit = new hitInfo();
  float dis = l_start.distanceTo(center);
  
  if(dis < r) {
    hit.hit = true;
    hit.t = -1;
    return hit;
  }
  
  //Step 2: Compute W - a displacement vector pointing from the start of the line segment to the center of the circle
    Vec2 toCircle = center.minus(l_start);
    
    //Step 3: Solve quadratic equation for intersection point (in terms of l_dir and toCircle)
    float a = 1;  //Lenght of l_dir (we noramlized it)
    float b = -2*dot(l_dir,toCircle); //-2*dot(l_dir,toCircle)
    float c = toCircle.lengthSqr() - r*r; //different of squared distances
    
    float d = b*b - 4*a*c; //discriminant 
    
    if (d >=0 ){ 
      //If d is positive we know the line is colliding, but we need to check if the collision line within the line segment
      //  ... this means t will be between 0 and the lenth of the line segment
      float t1 = (-b - sqrt(d))/(2*a); //Optimization: we only take the first collision [is this safe?]
      //println(hit.t,t1,t2);
      if (t1 > 0 && t1 < l_len && t1 < max_t){
        hit.hit = true;
        hit.t = t1;
      } 
    }
    
  return hit;
}




void mouseDragged() 
{
  boolean mouseinObstacle = false;
  for(int i = 0; i < numObstacles; i++) {
       if(pointInCircle(circlePos[i], circleRad[i],new Vec2(mouseX, mouseY) , 3)) {
         
         circlePos[i] = new Vec2(mouseX, mouseY);
         mouseinObstacle = true;
         break;
       }
  }
  if(!mouseinObstacle){
     objPos = new Vec2(mouseX, mouseY);
  
  }
}

void mouseClicked() {
 if (mouseButton == RIGHT){   //right click to place obstacle
    if(numObstacles + 1 <= maxNumObstacles) {
       circlePos[numObstacles] = new Vec2(mouseX, mouseY);//Circle positions
       circleRad[numObstacles] = (10+40*pow(random(1),3));  //Circle radii
       numObstacles ++;
    }
    else{
      circlePos[numObstacles-1] = new Vec2(mouseX, mouseY);
      circleRad[numObstacles-1] = (10+40*pow(random(1),3));
    }
 }

}
