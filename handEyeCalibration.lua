local xamla3d = require('xamla3d')


local xamlaHandEye = {}


local function pinv(x)
   local u,s,v = torch.svd(x,'A')
   local idx = torch.sum(torch.gt(s,0))
   local stm = s:pow(-1):narrow(1,1,idx)
   local n = stm:size()[1]
   local ss=torch.expand(torch.reshape(stm,n,1),n,n) -- for elementwise mult
   local vv = v:narrow(1,1,idx)
   local uu = u:narrow(1,1,idx)
   local pin = torch.mm(vv,torch.cmul(uu:t(),ss))
   return pin
end

--Hg - a table of 4x4 gripper poses
--Hc - a table of 4x4 camera positions (e.g. got from solvePnP)
--H - the estimated HandEye matrix
--return - A vector of alignment residuals
function xamlaHandEye.getAlignError(Hg, Hc, HandEye)
  local Tcg = HandEye[{{1,3},4}]

  assert(#Hg == #Hc)

  local Rcg = HandEye[{{1,3},{1,3}}]
  local Tcg = HandEye[{{1,3},{4}}]

  local Hg_ij = {}
  local Hc_ij = {}


  local nEquations = ((#Hg) * (#Hg-1))/2

  local coeff = torch.DoubleTensor(3*nEquations, 3)
  local const = torch.DoubleTensor(3*nEquations, 1)


  local cnt = 0

  for i = 1,#Hg do
    for j = i+1,#Hg do
      local dHg = torch.inverse(Hg[i]) * Hg[j]
      local dHc = Hc[i] * torch.inverse(Hc[j])
      table.insert(Hg_ij, dHg)
      table.insert(Hc_ij, dHc)
    end
  end



 for i = 1,#Hg_ij do
  coeff[{{(i-1)*3+1,(i-1)*3 + 3 }, {}}] = Hg_ij[i][{{1,3},{1,3}}] - torch.eye(3,3)
  const[{{(i-1)*3+1,(i-1)*3 + 3},  1}] = Rcg * Hc_ij[i][{{1,3},{4}}] - Hg_ij[i][{{1,3},{4}}]
 end

 local res = coeff * Tcg - const

 return torch.sum(torch.abs(res)), res

end


-- Perform hand Eye calibration using cross validation. i.e sample
-- nPoses randomly from Hg and Hc and calc hand eye. this hand eye matrix
-- is then evaluated according to its rotational alignment error
function xamlaHandEye.calibrateViaCrossValidation(Hg, Hc, nPoses, nTrials)

  assert(#Hg == #Hc)
  assert(nPoses >= 3)
  assert(nTrials >= 1)
  assert(#Hg >= nPoses)

  local minError = 10000
  local bestHESolution = nil
  local alignmentErrorTest = nil
  local alignmentError = nil

  for n = 1, nTrials do

  if (n % 100 == 0) then
    print(n)
  end

    local idx = torch.randperm(#Hg)
    local HgSamples,HcSamples = {},{}

    for i = 1, nPoses do
      table.insert(HgSamples, Hg[idx[i]])
      table.insert(HcSamples, Hc[idx[i]])
    end

    local HE, resAlignOpt, res_angle = xamlaHandEye.calibrate(HgSamples, HcSamples)

    print("maxTAlignment: " ..torch.max(resAlignOpt) .." MaxRAlignemnt:" ..torch.max(res_angle))
--    print("medianTAlignment: ")
--    print(torch.median(torch.squeeze(resAlignOpt)))
--    print(" medianRAlignemnt: ")

--    local rMedian, index = torch.max(torch.squeeze(res_angle))
--    print(" medianRAlignemnt: "..torch.squeeze(rMedian))

--    local rMax = torch.max(torch.squeeze(res_angle))
--    print(" medianRAlignemnt: "..rMax)

    if torch.max(res_angle) < 0.8 then
      local HgVal = {}
      local HcVal = {}

      for i = nPoses+1, #Hg do
        table.insert(HgVal, Hg[idx[i]])
        table.insert(HcVal, Hc[idx[i]])
      end

      local error, res = xamlaHandEye.getAlignError(HgVal, HcVal, HE)
      local resMean = torch.mean(res:abs())
      if resMean < minError then
        minError = resMean
        bestHESolution = HE:clone()
        print(string.format('new candidate (res: %f):', resMean))
        print(bestHESolution)
        alignmentErrorTest = res
        alignmentError = resAlignOpt
      end
    end
  end

  return bestHESolution, alignmentErrorTest, alignmentError
end


-- Hg: list of roboter poses
-- Hc: list of camera/pattern poses (from solvePnp for example)
-- Hc should be the camera pose in the pattern frame of reference
-- returns H = handPattern matrix: pose of the pattern in TCP coordinate frame
-- Algorithm based on paper Tsai and Lenz, 1987
function xamlaHandEye.calibrate(Hg, Hc)

  print('#Hg='..#Hg..' #Hc='..#Hc)
  assert(#Hg == #Hc)

  local Hg_ij = {}
  local Hc_ij = {}

  local Pg = {}
  local Pc = {}


  local nEquations = ((#Hg) * (#Hg-1))/2

  local coeff = torch.DoubleTensor(3*nEquations, 3)
  local const = torch.DoubleTensor(3*nEquations, 1)


  local cnt = 0

  -- calculate the difference between all pairs of two robot and two camera poses
  for i = 1,#Hg do
    for j = i+1,#Hg do

    local dHg = torch.inverse(Hg[j]) * Hg[i]  -- pose difference between two robot poses
    local dHc = Hc[j] * torch.inverse(Hc[i])  -- pose difference between the two corresponding camera poses

     table.insert(Hg_ij, dHg)
     table.insert(Hc_ij, dHc)


     local Pg_ij = xamlaHandEye.modRodrigues(dHg[{{1,3},{1,3}}])  -- convert rotation component to a kind of
     local Pc_ij = xamlaHandEye.modRodrigues(dHc[{{1,3},{1,3}}])  -- axis angle representation

     Pg_ij = Pg_ij:clone():view(3,1)
     Pc_ij = Pc_ij:clone():view(3,1)

     -- for explanation of the next two lines, see paper ;-)
     -- (creats a set of linear equations)
     coeff[{{cnt*3+1,cnt*3 + 3 }, {}}] = xamla3d.getSkewSymmetricMatrix(Pg_ij + Pc_ij)
     const[{{cnt*3+1, cnt*3 + 3},  1}] = Pc_ij:view(3,1) - Pg_ij:view(3,1)
     cnt = cnt+1

    end
  end

  -- solve the equations
  local AtA =  torch.DoubleTensor(3,3):zero()  
  print('coeff dimensions='..coeff:dim())
  local Atb = coeff:t() * const
  AtA = coeff:t() * coeff
  local Pcg_p = torch.inverse(AtA) * Atb


  local res_angle = coeff * Pcg_p - const

  -- calculate the rotation component
  local Pcg =  (Pcg_p * 2) / math.sqrt(1+torch.norm(Pcg_p)^2);

  local Rcg = xamlaHandEye.invModRodrigues(Pcg);

 coeff = torch.DoubleTensor(3*nEquations, 3):zero()
 const = torch.DoubleTensor(3*nEquations, 1):zero()

 -- --- process the translation component ---

 -- create a set of equations
 for i = 1,#Hg_ij do
  coeff[{{(i-1)*3+1,(i-1)*3 + 3 }, {}}] = Hg_ij[i][{{1,3},{1,3}}] - torch.eye(3,3)
  const[{{(i-1)*3+1,(i-1)*3 + 3},  1}] = Rcg * Hc_ij[i][{{1,3},{4}}] - Hg_ij[i][{{1,3},{4}}]
 end

 -- solve the set of equations
  local AtA =  torch.DoubleTensor(3,3):zero()
  local Atb = coeff:t() * const
  AtA = coeff:t() * coeff
  local Tcg = torch.inverse(AtA) * Atb

  local res = coeff * Tcg - const
  --print(torch.min(torch.abs(res)))
  --print(torch.max(torch.abs(res)))

  -- combine rotation and translation component to final result
  local H = torch.eye(4,4)
  H[{{1,3},{1,3}}] = Rcg
  H[{{1,3},4}] = Tcg

  return H, res, res_angle
end


local function unit(v)
    return v / torch.norm(v)
  end

function xamlaHandEye.modRodrigues(R)
  local P = xamla3d.rotMatrixToAxisAngle(R)
  local theta = torch.norm(P)

  if theta ~= 0 then
    P = unit(P) * 2 * math.sin(theta / 2)
  end
  return P
end


function xamlaHandEye.invModRodrigues(P)
  local R = torch.eye(3,3)

  if torch.norm(P) > 1e-14 then
    local theta =math.asin(torch.norm(P) / 2) * 2

    R = xamla3d.axisAngleToRotMatrix(unit(P) * theta)
  end

  return R

end




return xamlaHandEye
