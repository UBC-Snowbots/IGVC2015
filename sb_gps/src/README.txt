Function Documentation: 

Jan 7, 2015: 
-------------------------------------------------------------
Checklist: 

* Malloc for structs
* Free pointers 


Focus on sb_gps: 
Use pass by reference for all structs for easier abstraction 
Check NMEA parser in ros documentation 

Questions: 
Check for compass signal status, double? 
Check if x,y,z are needed for twist message 

Considerations: 
Change checkgoal to check d = 0 ? 

long/lat fomula: 

      double R = 6378.137; //radius of the earth in KM
      
      double dlong = (double)abs(flaglong-GPS.longitudeDegrees)*pi/180.0; 
      double dlat = (double)abs(flaglat-GPS.latitudeDegrees)*pi/180.0;
      double radian = (double)pi/180.0;

      double a = (double) sin(dlat/2.0)*sin(dlat/2.0)+cos(GPS.latitudeDegrees*pi/180.0)*cos(flaglat*pi/180.0)*sin(dlong/2.0)*sin(dlong/2.0);
      double c = (double) 2.0*(atan2(sqrt(a),sqrt(1-a))); 
      double d = (double) R*c;
      
      double x = (double)(flaglong-GPS.longitudeDegrees) * cos((flaglat+GPS.latitudeDegrees)/2);
      double y = (double)(flaglat-GPS.latitudeDegrees);
      double d = (double)(sqrt(x*x+y*y)*R);

-------------------------------------------------------------

Jan 8, 2015:
-------------------------------------------------------------
Questions: 
Message formats for twist? 

What is the characteristic line of code that makes a node accessible by rosrun?





