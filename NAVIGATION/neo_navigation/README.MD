******************************

  QBO_Neo_navigation package
      for ros melodic
          mai 2019

******************************

/cfg    : config files for /nodes utils

/nodes  : contain some utils for linear and angular calibration

     ex: rosrun QBO_Neo_navigation calibration_linear.py
         rosrun rqt_reconfigure rqt_reconfigure

       play with setups to improve odometry.

/maps   : contain created 2D maps

/3D_map : contain rtabmap 3D map as db file (rtabmap.db)

/params : config files for move_base and costmaps

/launch : launchs files for navigation

