from yade import pack

O.materials.append(FrictMat(density=2650,young=6.e8,poisson=.8,frictionAngle=.0))

sp = pack.SpherePack()
size = .15
sp.makeCloud(minCorner=(-size,-size,-size),maxCorner=(size,size,size),rMean=.01,rRelFuzz=.4,num=1000,periodic=True,seed=1)
sp.toSimulation()
O.cell.setBox((3*size,3*size,3*size))
print len(O.bodies)
# for p in O.bodies:
#    p.state.blockedDOFs = 'zXY'
#    p.state.mass = 2650 * 0.1 * pi * p.shape.radius**2 # 0.1 = thickness of cylindrical particle
#    inertia = 0.5 * p.state.mass * p.shape.radius**2
#    p.state.inertia = (.5*inertia,.5*inertia,inertia)
O.dt = utils.PWaveTimeStep()
print O.dt

Pt=PeriTriaxController(
      dynCell=True,
      goal=(-10.e3,-10.e3,-10.e3),
      stressMask=7,
      relStressTol=.01,
      maxUnbalanced=.01,
      maxStrainRate=(.5,.5,.5),
      doneHook='term()',
      label='biax'
   )

O.engines = [
   ForceResetter(),
   InsertionSortCollider([Bo1_Sphere_Aabb()]),
   InteractionLoop(
      [Ig2_Sphere_Sphere_ScGeom()],
      [Ip2_FrictMat_FrictMat_FrictPhys()],
      [Law2_ScGeom_FrictPhys_CundallStrack()]
   ),
   NewtonIntegrator(gravity=(-0,-0,-0), damping=.3)
]

def term():
   O.engines = O.engines[:3]+O.engines[4:]
   O.engines=O.engines+[PyRunner(command='addPlotData()',iterPeriod=200)]
   print getStress()
   print O.cell.hSize
   print O.engines
   setContactFriction(0.5)
   O.cell.trsf=Matrix3.Identity
   O.cell.velGrad=Matrix3.Zero
   
   for p in O.bodies:
      p.state.vel = Vector3.Zero
      p.state.angVel = Vector3.Zero
      p.state.refPos = p.state.pos
      p.state.refOri = p.state.ori
   O.save('0.yade.gz')
   O.pause()

#O.run();O.wait()

