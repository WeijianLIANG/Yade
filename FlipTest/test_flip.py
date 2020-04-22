from yade import plot
import numpy

O.load('0.yade.gz')
n = utils.porosity()
p=-100.e3

O.dt=1.e-5
rMean=3.e-3

stable_step=1000

def termi():
	#O.save('0_end.yade.gz')
	#plot.saveDataTxt('packing_0.txt',vars=('strainx','strainz','stressx','stressz','e'))
	O.pause()

PT=PeriTriaxController(
      dynCell=True,
      goal=(p,-500.,0),
      stressMask=1,
      relStressTol=1.e-2,
      maxUnbalanced=1.e-2,
      maxStrainRate=(0.6,0.2,0.00),
      doneHook='termi()',
      label='biax'
   )

Record=PyRunner(command='saveAddData()',iterPeriod=200)
FLIP=PyRunner(command='flip()',iterPeriod=200)
#setContactFriction(0.5)
#intrsOrigIds = [(i.id1,i.id2) for i in O.interactions]

Newton=O.engines[-1]

def saveAddData():
   maxx=0
   maxy=maxOverlapRatio()
   for i in O.bodies:
     if i.state.pos[0]>maxx:
		maxx=i.state.pos[0]

   stress = utils.getStress()
   strainx = -log(O.cell.trsf[0][0])
   strainy = -log(O.cell.trsf[1][1])
   stressy = -stress[1][1]
   stressx = -stress[0][0]
   n = utils.voidratio2D(0.1)
   stressRatio = stressy/stressx
   strainy_100= -log(O.cell.trsf[1][1])*100
   iter=O.iter
   plot.addData(strainx=strainx,
      strainy=strainy,
      stressy=stressy,
      stressx=stressx,
      n=n,
      stressRatio=stressRatio,
      strainy_100=strainy_100,
      maxx=maxx,
      maxy=maxy,
      iter=iter)



def flip():
	#O.cell.velGrad=Matrix3(0.,0.,0.,0,-0.,0,0,0,0)
	hSize=O.cell.hSize
	x_edge=hSize.col(0)
	y_edge=hSize.col(1)
	z_edge=hSize.col(2)

	flipCell()
	utils.calm()
	if x_edge.norm()>4.*y_edge.norm():
		print "Original:",hSize
		print "utils.voidratio2D(0.1)",utils.voidratio2D(0.1)
		current_hSize=Matrix3(0.5*x_edge+Vector3(rMean,0,0),2.*y_edge+Vector3(0,2.*rMean,0),z_edge)
		target_hSize=Matrix3(0.5*x_edge,2.*y_edge,z_edge)
		O.cell.hSize=current_hSize
		for i in O.bodies:
			nx = i.state.pos[0]//x_edge[0]
			ny = i.state.pos[1]//y_edge[1]
			newx=i.state.pos[0]-x_edge[0]*nx
			newy=i.state.pos[1]-y_edge[1]*ny
			i.state.pos=Vector3(newx,newy,i.state.pos[2])

			x=i.state.pos[0]%x_edge[0]
			y=i.state.pos[1]%y_edge[1]
			
			if x>0.5*x_edge[0]:
				i.state.pos=i.state.pos-x_edge*0.5+y_edge+Vector3(0.,rMean,0.)
			
		for i in O.interactions.all():
		  	i.cellDist=Vector3i.Zero
		O.engines=[ForceResetter(),
		InsertionSortCollider([Bo1_Sphere_Aabb()],sortThenCollide=True,label='collider'),
		InteractionLoop(
			[Ig2_Sphere_Sphere_ScGeom()],
			[Ip2_FrictMat_FrictMat_FrictPhys()],
			[Law2_ScGeom_FrictPhys_CundallStrack()]
		)]+[Newton]
		#flipCell()
		O.cell.velGrad=Matrix3.Zero
		
		d_hSize=(current_hSize-target_hSize)/stable_step
		#d_hSize=Matrix3.Zero
		O.pause()
		for i in range(stable_step):
			O.cell.hSize-=d_hSize
			O.step()
		
		O.engines[0]()
		O.engines=O.engines[:3]+[
		PT,Record,FLIP,Newton	
		] 
		
		print "After flip:",O.cell.hSize
		print "utils.voidratio2D(0.1)",utils.voidratio2D(0.1)
		O.run()
		
	




#O.engines += [PyRunner(command='saveAddData()',iterPeriod=300)]
#biax.goal=(-5.e4,-1.2e-1,0)
#biax.stressMask=1
#biax.relStressTol=0.01
#biax.maxUnbalanced=0.01
#biax.maxStrainRate=(1.,.1,0.)
#biax.doneHook='termi()'

O.engines = [ForceResetter(),
	InsertionSortCollider([Bo1_Sphere_Aabb()],label='collider'),
	InteractionLoop(
		[Ig2_Sphere_Sphere_ScGeom()],
		[Ip2_FrictMat_FrictMat_FrictPhys()],
		[Law2_ScGeom_FrictPhys_CundallStrack()]
	),
   PT,
   Record,
   FLIP,
   Newton	
] 

#mask sure the white space after 'strainz_100', it used to differentiate entries
plot.plots={'strainy_100 ':('stressy','stressx'),'strainy_100  ':('n'),'iter':('stressx','stressy')}
plot.plot()


# O.run()
