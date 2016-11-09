#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>


class SpecificWorker : public GenericWorker
{
 Q_OBJECT

	public:
		SpecificWorker(MapPrx& mprx);	
		~SpecificWorker();
		bool setParams(RoboCompCommonBehavior::ParameterList params);
		void setPick(const Pick &mypick);
		
		virtual void go(const string &nodo, const float x, const float y, const float alpha);
		virtual void turn(const float speed);
		virtual bool atTarget();
		virtual void stop();

	public slots:
		void compute(); 	

	private:    
		enum class State {INIT,GOTO,BUG,END, BUGINIT};
		struct Target
		{
			mutable QMutex m;
			QVec pose = QVec::zeros(3);
			float angl;
			bool active = false;
			void setActive(bool newActive)
			{
				QMutexLocker lm(&m);
				active = newActive;
			}
			void copy(float x, float z)
			{
				QMutexLocker lm(&m);
				pose.setItem(0,x);
				pose.setItem(1,0);
				pose.setItem(2,z);
			}
			QVec getPose()
			{
				QMutexLocker lm(&m);
				return pose;
			}
		};
		InnerModel* innermodel;
		State state = State::INIT;
		Target t;
		QLine2D linea;
		float distanciaAnterior;
		void dodge(int threshold,RoboCompLaser::TLaserData ldata);
		void move(const TLaserData &tLaser);
		bool obstacle(TLaserData tLaser);
		void bug( const TLaserData& ldata, const TBaseState& bState );
		bool targetAtSight(TLaserData ldata);
		void buginit( const TLaserData& ldata, const TBaseState& bState );
		void stopRobot();
		float obstacleLeft( const TLaserData& tlaser);
		float distanceToLine(const TBaseState& bState);
	};
	
#endif
