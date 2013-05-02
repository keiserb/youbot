This is a manual on how to get the Gravity Compensation of the arm with Orocos (http://youbot-store.com/apps-and-utilities/32/software.aspx) to work on the ailab hardware. 

After following the installation instructions you will notice that there appear several errors when trying to run the software. This is due to the fact that the ailab YouBot is a newer model with updated drives and there are some issues with compatibility in Lua. Make sure you also check out http://www.orocos.org/wiki/KukaYoubot for additional infos.

Error 1: TaskContext.getPeer: no peer deployer
This comes from an upgrade in Orocos which Lua didn't implement yet where deployer got renamed to Deployer. So simply rename deployer to Deployer in all the .lua scripts that cause this issue.

Error 2: 
> 4.163 [ ERROR  ][youbot] Wheel slave 2: TMCM-174 not detected, found
> TMCM-1632instead.
> 4.166 [ ERROR  ][youbot] Arm power board: KR-843 not detected, found
> KR-845instead.
> 4.166 [ ERROR  ][youbot] Arm power board: KR-843 not detected, found
> KR-845instead.
This comes from the updated YouBot hardware, namely the Ethercat slaves:
TMCM-1610 <-> TMCM-KR-841 (x5, one for each arm joint)
TMCM-1632 <-> TMCM-174 ( x4, one for each wheel)
In the file youbot_driver_rtt/src/youbot_types.hpp change the corresponding lines, e.g:
//#define YOUBOT_WHEELCONTROLLER_SLAVENAME "TMCM-174"
#define YOUBOT_WHEELCONTROLLER_SLAVENAME "TMCM-1632"
//#define YOUBOT_JOINTCONTROLLER_SLAVENAME "TMCM-KR-841"
#define YOUBOT_JOINTCONTROLLER_SLAVENAME "TMCM-1610"

Error 3:
> ./opt/ros/fuerte/stacks/orocos_toolchain/ocl/bin/rttlua-gnulinux: ...e/stacks/orocos_toolchain/ocl/lua/modules/rttros.lua:26: Package path could not be found for youbot_driver_rtt
> stack traceback:
>     [C]: in function 'assert'
>     ...e/stacks/orocos_toolchain/ocl/lua/modules/rttros.lua:26: in function 'find_rospack'
>     lua/youbot_test.lua:115: in main chunk
>     [C]: ?
> TLSF bytes allocated=524288 overhead=3248 max-used=3248 currently-used=3248 still-allocated=0
Is caused by an issue of fuerte and a setcap'ed binary.
As a fix, replace the contents of the orocos_toolchain/ocl/lua/modules/rttros.lua with the following

module("rttros", package.seeall)

local rospack_loaded=false
function find_rospack(package)
   if not rospack_loaded then
      if not (rtt and rttlib) then
         error("find_rospack: not an rttlua _or_ rttlib not loaded.")
      end
      depl = rttlib.findpeer("deployer") or rttlib.findpeer("Deployer")
      if not depl then error("find_rospack: failed to find a deployer") end
      depl:import("rtt_rospack")
      rospack_loaded=true
   end
   return rtt.provides("rospack"):find(package)
end

-- Help Markus' poor, confused brain:
rospack_find=find_rospack

This should solve all the issues and the program should run without a problem now.