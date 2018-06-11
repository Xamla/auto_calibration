local lfs = require 'lfs'
local cv = require 'cv'
require 'cv.highgui'
require 'cv.videoio'
require 'cv.imgproc'
require 'cv.calib3d'
require 'cv.imgcodecs'
require 'cv.features2d'

local xamla3d = {}
xamla3d.utils = {}
xamla3d.calibration = {}

-- returns the intersection of two lines in 2D. Line 1 is defined two points (p1 and p2) and
-- Line 2 by (p3 and p4) 
function xamla3d.intersectLines (p1,p2, p3, p4)
  local x_nom = (p1.x*p2.y - p1.y*p2.x)*(p3.x-p4.x) - (p1.x - p2.x)*(p3.x*p4.y-p3.y*p4.x)
  local y_nom = (p1.x*p2.y - p1.y*p2.x)*(p3.y-p4.y) - (p1.y - p2.y)*(p3.x*p4.y-p3.y*p4.x)
  local denom = (p1.x - p2.x)*(p3.y-p4.y) -(p1.y-p2.y)*(p3.x-p4.x)
  local X = {}
  X.x = x_nom / denom
  X.y = y_nom / denom
  return X
end



-- Returns the distance of a point to a line defined by two points P1 and P2
function xamla3d.pointLineDistance (P1, P2, X)
 local nom = (P2.y - P1.y)*X.x - (P2.x-P1.x)*X.y + P2.x*P1.y-P2.y*P1.x
 local denom = math.sqrt((P2.y+P1.y)^2 + (P2.x + P2.y)^2)
 return math.abs(nom / denom)
end


function xamla3d.axisAngleToRotMatrix(vec)
   local vec_3x1 = vec:view(3,1):clone()
  local theta = torch.norm(vec_3x1)
  local R
  if (theta < 1e-14) then
    R = torch.eye(3,3)
  else
    local alpha = math.cos(theta)
    local beta = math.sin(theta)
    local gamma = 1-math.cos(theta)
    
    local omega = vec_3x1 / theta
    local omegav = xamla3d.getSkewSymmetricMatrix(omega:view(3,1)) 
    
     local A = omega*omega:t();
  
     R = torch.eye(3,3)*alpha + omegav*beta + A*gamma;
  end

  return R
  
end

function xamla3d.rotMatrixToAxisAngle(rotation_3x3)
 local R = rotation_3x3:clone()
 local U,S,V = torch.svd(R)
 R = U * V:t()
 local tr = (torch.trace(R)-1)/2 
 local theta = math.acos(tr)
 local out
 if (math.sin(theta) >= 1e-12) then    
  local vth = 1/(2*math.sin(theta));  
  local om1 = torch.DoubleTensor({R[3][2]-R[2][3], R[1][3]-R[3][1], R[2][1]-R[1][2]}):view(3,1):t():clone();  
  local om = om1 * vth;
  out = om*theta;
 else
   if tr > 0 then       -- case norm(om)=0;          
       print("Case1")  
       print(R)
      out = torch.DoubleTensor(3):zero();
   else     
     print("Case2")
     local sign = torch.DoubleTensor(3,1)
     sign[1][1] = 1
     sign[{{2,3}, 1}] = (((R[{1,{2,3}}]:ge(0))*2):type('torch.DoubleTensor'))-1 
     out = ((torch.sqrt((torch.diag(R)+1)/2)):cmul(sign))* theta             
   end   
 end
 
 return out
 
end

--Returns the fundamental matrix given two 3x4 projection matrices
function xamla3d.getFundamentalMatrix (K, R1, t1, R2, t2)
  --let K1 and K2 there (if we have later different K's
  local K1 = K
  local K2 = K

  local R = R2 * R1:t();
  local t = t2 - R * t1;

  local T = xamla3d.getSkewSymmetricMatrix(t)
  local E = T * R;

  local  F = torch.inverse(K2:t()) * E * torch.inverse(K1);
  return F
end


function xamla3d.getFundamentalMatrixFromFullPose (Pose1_4x4, Pose2_4x4, intrinsic_3x3)
  local R1 = Pose1_4x4[{{1,3},{1,3}}]
  local R2 = Pose2_4x4[{{1,3},{1,3}}]
  local t1 = Pose1_4x4[{{1,3},{4}}]
  local t2 = Pose2_4x4[{{1,3},{4}}]
  local F = xamla3d.getFundamentalMatrix(intrinsic_3x3, R1, t1, R2, t2)
  return F
end


function xamla3d.sampsonDistance (P1_3x1, P2_3x1, F_3x3)
  local nom = P2_3x1:t() * F_3x3 * P1_3x1
  nom = nom*nom

  local denom_t1 = ((F_3x3*P1_3x1)[1])
  local denom_t2 = ((F_3x3*P1_3x1)[2])
  local denom_t3 = ((F_3x3:t()*P2_3x1)[1])
  local denom_t4 = ((F_3x3:t()*P2_3x1)[2])

  return nom[1][1] / (denom_t1[1]^2 + denom_t2[1]^2 + denom_t3[1]^2 + denom_t4[1]^2)
end


function xamla3d.shuffleTable( t )
    math.randomseed(os.time())
    local rand = math.random 
    assert(t, "shuffleTable() expected a table, got nil")
    local iterations = #t
    local j
    
    for i = iterations, 2, -1 do
      j = rand(i)
      t[i], t[j] = t[j], t[i]
    end
end


function xamla3d.drawEpipolarLineWithF(F, image2, pts2d_image1_2xX, image1)
   local lines = cv.computeCorrespondEpilines{points = pts2d_image1_2xX, whichImage = 1, F = F}

  local w = image2:size()[2]

  for i = 1,lines:size()[1] do
    local l = lines[{i,{}}];
    local ep1_y = (l[1][1]*0+l[1][3]) / -l[1][2]
    local ep2_y = (l[1][1]*w+l[1][3]) / -l[1][2]

    local c = torch.rand(3)*255
    c = c:type('torch.ByteTensor')
        
    if image1 ~= nil then
      
      local center = {pts2d_image1_2xX[{i, 1, 1}], pts2d_image1_2xX[{i,1, 2}]}
  
      cv.circle{img = image1, center = center, radius = 6, color = {c[1],c[2],c[3]}, thickness = 1, lineType = cv.LINE_AA}
    end
        

    cv.line{img = image2, pt1 = {x = 0, y = ep1_y}, pt2 = {x = w, y = ep2_y}, thickness = 1, color = {c[1],c[2],c[3]}}
  end
end

function xamla3d.drawEpipolarLine (Pose1_4x4, Pose2_4x4, intrinsic_3x3, image2, pts2d_image1_2xX)
  local F = xamla3d.getFundamentalMatrixFromFullPose(Pose1_4x4, Pose2_4x4, intrinsic_3x3)

  local lines = cv.computeCorrespondEpilines{points = pts2d_image1_2xX, whichImage = 1, F = F}

  local w = image2:size()[2]

  for i = 1,lines:size()[1] do
    local l = lines[{i,{}}];
    local ep1_y = (l[1][1]*0+l[1][3]) / -l[1][2]
    local ep2_y = (l[1][1]*w+l[1][3]) / -l[1][2]
    
    local color = torch.rand(3,1)*255
    
    cv.line{img = image2, pt1 = {x = 0, y = ep1_y}, pt2 = {x = w, y = ep2_y}, thickness = 1, color = {0,243,255}}
  end
end


-- returns the skew symmetric matrix from a vector
function xamla3d.getSkewSymmetricMatrix (vec)
  local S = torch.zeros(3,3)
  S[1][2] = -vec[3]
  S[1][3] =  vec[2]

  S[2][1] =  vec[3]
  S[2][3] = -vec[1]

  S[3][1] = -vec[2]
  S[3][2] =  vec[1]
  return S
end


-- P: table of 3x4 projection matrices
-- measurements: table of measurements, where each measurement is a 1x2 torch Tensor
-- X the 3d point
function xamla3d.getReprojectionError (P,measurements, X)
  local re = torch.DoubleTensor(#P,1)  

  local X_h = torch.DoubleTensor(4,1)
  X_h[1] = X[1]
  X_h[2] = X[2]
  X_h[3] = X[3]
  X_h[4] = 1

  for i = 1,#P do
    local x = P[i][{{1,3}, {}}]*X_h
    x = x  * (1 / x[3][1])
    
    local rx = (measurements[i][1][1] - x[1][1])^2
    local ry = (measurements[i][1][2] - x[2][1])^2
    local d = math.sqrt(rx+ry)
    
    re[i][1] =  d    
  end
  
  return re, torch.sum(re)
end

function xamla3d.ransacTriangulation (P, measurements, reThreshold, maxIterations)
  if (#P < 2) then
    return false
  end
  
  local n = #measurements
  
  local maxInlier = 0
  local bestX = nil
  
  for i = 1,maxIterations do
    local idx = torch.randperm(n)

    local P_ = {}
    local meas_ = {}
    for j = 1,2 do
      table.insert(P_, P[idx[j]])
      table.insert(meas_, measurements[idx[j]])
    end
          
    local s, X = xamla3d.linearTriangulation(P_,meas_)    
    local re, sum = xamla3d.getReprojectionError(P,measurements,X)       
    
    local inlier = torch.sum(torch.le(re, reThreshold))
    if inlier > maxInlier then
      maxInlier = inlier
      bestX = X:clone()
    end
  end
  
  return true, bestX, maxInlier
end


-- P is a table of 3x4 projection matrices
-- measurement is a table with 1x2 image measurements
function xamla3d.linearTriangulation (P, measurements)
  if #P ~= #measurements then
    return false, nil
  end

  local m = #measurements

  local A = torch.zeros(2*#measurements,3)
  local B = torch.zeros(2*#measurements,1)

  for i = 1,#P do
    local p = P[i]
    local x = measurements[i][1][1]
    local y = measurements[i][1][2]

    A[{(i-1)*2+1, 1}] = p[{3,1}]*x - p[{1,1}]
    A[{(i-1)*2+1, 2}] = p[{3,2}]*x - p[{1,2}]
    A[{(i-1)*2+1, 3}] = p[{3,3}]*x - p[{1,3}]

    A[{(i-1)*2+2, 1}] = p[{3,1}]*y - p[{2,1}]
    A[{(i-1)*2+2, 2}] = p[{3,2}]*y - p[{2,2}]
    A[{(i-1)*2+2, 3}] = p[{3,3}]*y - p[{2,3}]

    B[{(i-1)*2+1,1}] = p[{1,4}] - x * p[{3,4}]
    B[{(i-1)*2+2,1}] = p[{2,4}] - y * p[{3,4}]
  end
  
  local AtA = torch.zeros(3, 3)
  local Atb = A:t() * B
  AtA = A:t() * A
  local X = torch.inverse(AtA) * Atb

  return true, X
end


-- Projects a point X (in World coordinates) to camera P 
function xamla3d.projectPoint (K, Rt, X)  

  local Xw = torch.zeros(4)
  Xw:view(4,1)[{{1,3},1}] = X:view(3,1)[{{1,3},1}]  
  Xw[4] = 1
  local xc = K *Rt[{{1,3},{1,4}}]*Xw
  local xc = xc / xc[3]
  return xc:view(3,1)[{{1,2},1}]
end


function xamla3d.calibration.calcPatternPointPositions (pointsX, pointsY, pointSize)
  -- Input params:
  --  pointsX  -- number of points in horizontal direction
  --  pointsY  -- number of points in vertical direction
  --  pointSize -- size of one point in mm

  -- calculates the "true" x, y, z position of the circle pattern
  local corners = torch.FloatTensor(pointsX*pointsY, 1, 3):zero()
  local i=1
  for y=1, pointsY do
    for x=1, pointsX do
      corners[i][1][1] = (2*(x-1) + (y-1)%2) * pointSize
      corners[i][1][2] = (y-1)*pointSize
      corners[i][1][3] = 0
      i = i+1
    end
  end
  --print(i..", "..arg.pointsX*arg.pointsY)
  return corners
end


function xamla3d.calibration.getPoseFromTarget (image, intrinsics, distCoeffs, patternSize, distCircles, cvFlags)

  local points3d = xamla3d.calibration.calcPatternPointPositions(patternSize.width,patternSize.height, distCircles)
  local found
  local pattern
  if cvFlags then
     found, pattern = xamla3d.calibration.findPattern(image, cv.CALIB_CB_ASYMMETRIC_GRID+cvFlags, patternSize)
  else
     found, pattern = xamla3d.calibration.findPattern(image, cv.CALIB_CB_ASYMMETRIC_GRID, patternSize)
  end

  if found then
    local poseFound, poseCamRotVector, poseCamTrans=cv.solvePnP{objectPoints=points3d, imagePoints=pattern, cameraMatrix=intrinsics, distCoeffs=distCoeffs}  
    local poseCamRotMatrix = xamla3d.calibration.RotVectorToRotMatrix(poseCamRotVector)
    local transfer = torch.DoubleTensor(4,4):zero()
    transfer[{{1,3}, {1,3}}] = poseCamRotMatrix
    transfer[{{1,3}, {4}}] = poseCamTrans
    transfer[4][4]=1
    return poseFound, transfer, pattern
  end
  return false, nil

end


-- Open one image file, search for the circle pattern and extract the positions of the circle centers
-- If the pattern is not found in the first try, image contract and brightness is increased up to 4 times (Introduced 
--   to be able to process dark images from the IR cam)
function xamla3d.calibration.findPattern (image, patternType, patternSize)
  -- Input params:
  --  image
  --  patternType -- type of the image pattern (valid: cv.CALIB_CB_ASYMMETRIC_GRID, other not tested)
  --  patternSize -- number of dots in x and y, e.g. patternGeom = {height=11, width=4}
  
  -- Return values
  --  bool: pattern recognition was successful (true) or some error/problem occured (false)
  --  torch.FloatTensor:  containing the coordinates of the pattern points (2D)
  
  local pointFindSuccess
  local centers
  for tryCounter=1, 4 do
    pointFindSuccess, centers=cv.findCirclesGrid{image=image, patternSize=patternSize, flags=patternType}
    if(not pointFindSuccess) then
      image:apply(function(x) local val=x*1.2+10 if(val>255) then return 255 else return val end end)
      print("Increasing imge brightness ...")   
      --cv.imshow{"Tmp", image}
      --cv.waitKey{-1}
    else
      break
    end
  end
    
  if(pointFindSuccess == true) then 
    if(0) then 
      for i = 1, (#centers)[1] do
        local point = cv.Point(centers[i][1][1], centers[i][1][2])
        cv.circle{img = image, center = point, radius = 6, color = {80,80,255,1}, thickness = 1, lineType = cv.LINE_AA}
        cv.circle{img = image, center = point, radius = 1, color = {0,250,250,1}, thickness = 1.2, lineType = cv.LINE_AA}
      end
      --cv.imshow{"edges", image}
      --cv.waitKey{-1}
    end
    return true, centers
  else
    print("Calibration pattern not found in image ")
	 --cv.imshow{"edges", image}
    --cv.waitKey{-1}
    return false, nil
  end
end


function xamla3d.calibration.RotVectorToRotMatrix (vec)
  -- transform a rotation vector as e.g. provided by solvePnP to a 3x3 rotation matrix using the Rodrigues' rotation formula
  -- see e.g. http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20Rodrigues%28InputArray%20src,%20OutputArray%20dst,%20OutputArray%20jacobian%29

  local theta = torch.norm(vec)
  local r = vec/theta
  r = torch.squeeze(r)
  local mat = torch.Tensor({{0, -1*r[3], r[2]}, {r[3], 0, -1*r[1]}, {-1*r[2], r[1], 0}})
  r = r:resize(3,1)

  local result = torch.eye(3)*math.cos(theta) + (r*r:t())*(1-math.cos(theta)) + mat*math.sin(theta)

  return result
end


function xamla3d.utils.readFileList (directory, prefix, postfix)
  if not xamla3d.utils.isDir(directory) then
    return false 
  end
  
  local files = {}
  for file in lfs.dir(directory) do
    if (string.find(file,prefix) and string.find(file, postfix)) then
      table.insert(files, directory..'/'..file)
    end
  end
  
  table.sort(files)
  return true, files
end


-- given two lists of files that contain a number, this function returns the corresponding files
-- Idea: extract a number from the first filename.
-- Go through the second list and have a look if there is a file with an identical number, e.g.
-- file_000345.png matches to rgb_000345_depth.stereo.jpg
function xamla3d.utils.matchNamesByIdenticalNumber (listOfNames1, listOfNames2)

  local matches = {}

  for i=1,#list do
    local v = listOfNames1[i]
    local number = string.match(v, "%d+")
    for j=1,#listOfNames2 do
      if (string.match(listOfNames2[j], number)) then
        table.insert(matches, {listOfNames1[i], listOfNames2[j]})
      end
    end
  end

  return matches
end


function xamla3d.utils.printMatrixAsTorchTensor(m, name)
if m:dim() ~= 2 then
 return
end

local write = io.write

write(name .." = torch.DoubleTensor({\n")
for i = 1,m:size()[1] do
  write("{")
  for j = 1,m:size()[2] do
  write(m[i][j].. ",")
  end
  write("},\n")
end

write("})\n")

end

function xamla3d.utils.isFile(name)
  if type(name)~="string" then return false end
  if not isDir(name) then
    return os.rename(name,name) and true or false
    -- note that the short evaluation is to
    -- return false instead of a possible nil
  end
  return false
end


function xamla3d.utils.getFilenameFromPath(path)
  local path, file, ext  = string.match(path, "(.-)([^\\/]-%.?([^%.\\/]*))$")
  return file
end


function xamla3d.utils.isFileOrDir(name)
  if type(name)~="string" then return false end
  return os.rename(name, name) and true or false
end


function xamla3d.utils.isDir(name)
  if type(name)~="string" then return false end
  local cd = lfs.currentdir()
  local is = lfs.chdir(name) and true or false
  lfs.chdir(cd)
  return is
end


-- writes an image to a file using opencv imwrite function
-- if directory does not exist, the directory is created
function xamla3d.writeImageToFile(directory, filename, image)  
  if not xamla3d.utils.isDir(directory) then
    lfs.mkdir(directory)
  end
  
  cv.imwrite{directory.."/"..filename, image}
end

return xamla3d
