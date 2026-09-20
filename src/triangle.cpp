#include "triangle.h"

Triangle::Triangle(Vector c, Vector b, Vector a, Texture* t):Plane(Vector(0,0,0), t, 0., 0., 0., 0., 0.),va(a),vb(b),vc(c){
   center = c;
   Vector righta = (b-c);
   textureX = righta.mag();
   right = righta/textureX;
   vect = right.cross(b-a).normalize();

   xsin = -right.z;
   if(xsin<-1.)xsin = -1;
   else if (xsin>1.)xsin=1.; 
   yaw = asin(xsin);
   xcos = sqrt(1.-xsin*xsin);

   zcos = right.x/xcos;
   zsin = -right.y/xcos;
   if(zsin<-1.)zsin = -1;
   else if (zsin>1.)zsin=1.;
   if(zcos<-1.)zcos = -1;
   else if (zcos>1.)zcos=1.;
   roll = asin(zsin);

   ycos = vect.z/xcos;
   if(ycos<-1.)ycos = -1;
   else if (ycos>1.)ycos=1.;
   pitch = acos(ycos);
   ysin = sqrt(1-ycos*ycos);

   up.x = -xsin*ysin*zcos+ycos*zsin;
   up.y = ycos*zcos+xsin*ysin*zsin;
   up.z = -xcos*ysin;
   Vector temp = vect.cross(right);
   Vector np = solveScalers(right, up, vect, a-c);
   textureY = np.y;
   thirdX = np.x;
   
   d = -vect.dot(center);
}

// double Triangle::getIntersection(Ray ray){
//    double time = Plane::getIntersection(ray);
//    if(time==inf) 
//       return time;
//    Vector dist = solveScalers(right, up, vect, ray.point+ray.vector*time-center); 
//    unsigned char tmp = (thirdX - dist.x) * textureY + (thirdX-textureX) * (dist.y - textureY) < 0.0;
//    return((tmp!=(textureX * dist.y < 0.0)) || (tmp != (dist.x * textureY - thirdX * dist.y < 0.0)))?inf:time;
// }
double Triangle::getIntersection(Ray ray){
  Vector edge1 = vb - vc;
  Vector edge2 = va - vc;

  const Vector ray_cross_e2 = ray.vector.cross(edge2);
  const double det = edge1.dot(ray_cross_e2);

  if (fabs(det) < 1e-12) return inf;   // ray parallel to triangle

  const double inv_det = 1.0 / det;
  Vector s = ray.point - vc;
  const double u = inv_det * s.dot(ray_cross_e2); 

  if (u < 0. || u > 1.) return inf;    // outside edge v0-v1

  const Vector s_cross_e1 = s.cross(edge1);
  const double v = inv_det * ray.vector.dot(s_cross_e1);

  if (v < 0. || u + v > 1.) return inf; // outside edge v0-v2

  const double t = inv_det * edge2.dot(s_cross_e1);
  return (t > 0.) ? t : inf;
}

bool Triangle::getLightIntersection(Ray ray, double* fill){
   const double t = ray.vector.dot(vect);
   const double norm = vect.dot(ray.point)+d;
   const double r = -norm/t;
   if(r<=0. || r>=1.) return false;
   Vector dist = solveScalers(right, up, vect, ray.point+ray.vector*r-center);
   
   unsigned char tmp = (thirdX - dist.x) * textureY + (thirdX-textureX) * (dist.y - textureY) < 0.0;
   if ((tmp!=(textureX * dist.y < 0.0)) || (tmp != (dist.x * textureY - thirdX * dist.y < 0.0))) return false;
   
   if(texture->opacity>1-1E-6) return true;   
   unsigned char temp[4];
   double amb, op, ref;
   texture->getColor(temp, &amb, &op, &ref,fix(dist.x/textureX-.5), fix(dist.y/textureY-.5));
   if(op>1-1E-6) return true;
   fill[0]*=temp[0]/255.;
   fill[1]*=temp[1]/255.;
   fill[2]*=temp[2]/255.;
   return false;
}
