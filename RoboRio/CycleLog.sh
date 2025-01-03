#!/bin/sh
# This file should get called from FRCRunRobot.sh before starting
# robot program
 
cd /home/lvuser

if test -f FRC_UserProgram.log
then
  if test -f FRC_UserProgram7.log
  then rm FRC_UserProgram7.lop
  fi
  if test -f FRC_UserProgram6.log
  then mv FRC_UserProgram6.log FRC_UserProgram7.log
  fi
  if test -f FRC_UserProgram5.log
  then mv FRC_UserProgram5.log FRC_UserProgram6.log
  fi
  if test -f FRC_UserProgram4.log
  then mv FRC_UserProgram4.log FRC_UserProgram5.log
  fi
  if test -f FRC_UserProgram3.log
  then mv FRC_UserProgram3.log FRC_UserProgram4.log
  fi
  if test -f FRC_UserProgram2.log
  then mv FRC_UserProgram2.log FRC_UserProgram3.log
  fi
  if test -f FRC_UserProgram.log
  then cp FRC_UserProgram.log FRC_UserProgram2.log
  fi
  chmod 666 FRC_UserProgram?.log
fi
      
exit 0
