/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the
use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but is not required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
Initial GJK-EPA collision solver by Nathanael Presson, 2008
Improvements and refactoring by Erwin Coumans, 2008-2014
*/
#if ! BT_GJK_EPA3_H
#define BT_GJK_EPA3_H

#include "LinearMath/btTransform.h"
#include "btGjkCollisionDescription.h"



struct	btGjkEpaSolver3
{
struct	sResults
	{
	enum eStatus
		{
		Separated,		/* Shapes doesnt penetrate												*/ 
		Penetrating,	/* Shapes are penetrating												*/ 
		GJK_Failed,		/* GJK phase fail, no big issue, shapes are probably just 'touching'	*/ 
		EPA_Failed		/* EPA phase fail, bigger problem, need to save parameters, and debug	*/ 
		}		status;
	btVector3	witnesses[2];
	btVector3	normal;
	double	distance;
	};


};



#if defined(DEBUG) || defined (_DEBUG)
#include <stdio.h> //for debug Console.WriteLine
#if __SPU__
#include <spu_printf.h>
#define Console.WriteLine spu_printf
#endif //__SPU__
#endif


    
    // Config
    
    /* GJK	*/
#define GJK_MAX_ITERATIONS	128
#define GJK_ACCURARY		((double)0.0001)
#define GJK_MIN_DISTANCE	((double)0.0001)
#define GJK_DUPLICATED_EPS	((double)0.0001)
#define GJK_SIMPLEX2_EPS	((double)0.0)
#define GJK_SIMPLEX3_EPS	((double)0.0)
#define GJK_SIMPLEX4_EPS	((double)0.0)
    
    /* EPA	*/
#define EPA_MAX_VERTICES	64
#define EPA_MAX_FACES		(EPA_MAX_VERTICES*2)
#define EPA_MAX_ITERATIONS	255
#define EPA_ACCURACY		((double)0.0001)
#define EPA_FALLBACK		(10*EPA_ACCURACY)
#define EPA_PLANE_EPS		((double)0.00001)
#define EPA_INSIDE_EPS		((double)0.01)
    
    
    // Shorthands
    typedef uint	U;
    typedef byte	U1;
    
    // MinkowskiDiff
    template <typename btConvexTemplate>
    struct	MinkowskiDiff
    {
        btConvexTemplate* m_convexAPtr;
        btConvexTemplate* m_convexBPtr;
        
        btMatrix3x3				m_toshape1;
        btTransform				m_toshape0;
        
        bool					m_enableMargin;
        
        
        MinkowskiDiff(btConvexTemplate& a, btConvexTemplate& b)
        :m_convexAPtr(&a),
        m_convexBPtr(&b)
        {
        }
        
        void					EnableMargin(bool enable)
        {
            m_enableMargin = enable;
        }
        inline btVector3		Support0(ref btVector3 d)
        {
            return m_convexAPtr.getLocalSupportWithMargin(d);
        }
        inline btVector3		Support1(ref btVector3 d)
        {
            return m_toshape0*m_convexBPtr.getLocalSupportWithMargin(m_toshape1*d);
        }
        
        
        inline btVector3		Support(ref btVector3 d)
        {
            return(Support0(d)-Support1(-d));
        }
        btVector3				Support(ref btVector3 d,U index)
        {
            if(index)
                return(Support1(d));
            else
                return(Support0(d));
        }
    };
    
enum	eGjkStatus
{
    eGjkValid,
    eGjkInside,
    eGjkFailed
};

    // GJK
    template <typename btConvexTemplate>
    struct	GJK
    {
        /* Types		*/
        struct	sSV
        {
            btVector3	d,w;
        };
        struct	sSimplex
        {
            sSV*		c[4];
            double	p[4];
            U			rank;
        };
        
        /* Fields		*/
        
        MinkowskiDiff<btConvexTemplate>			m_shape;
        btVector3		m_ray;
        double		m_distance;
        sSimplex		m_simplices[2];
        sSV				m_store[4];
        sSV*			m_free[4];
        U				m_nfree;
        U				m_current;
        sSimplex*		m_simplex;
        eGjkStatus      m_status;
        /* Methods		*/
        
        GJK(btConvexTemplate& a, btConvexTemplate& b)
        :m_shape(a,b)
        {
            Initialize();
        }
        void				Initialize()
        {
            m_ray		=	btVector3(0,0,0);
            m_nfree		=	0;
            m_status	=	eGjkFailed;
            m_current	=	0;
            m_distance	=	0;
        }
        eGjkStatus			Evaluate(string inkowskiDiff<btConvexTemplate>& shapearg,ref btVector3 guess)
        {
            U			iterations=0;
            double	sqdist=0;
            double	alpha=0;
            btVector3	lastw[4];
            U			clastw=0;
            /* Initialize solver		*/
            m_free[0]			=	m_store;
            m_free[1]			=	&m_store[1];
            m_free[2]			=	&m_store[2];
            m_free[3]			=	&m_store[3];
            m_nfree				=	4;
            m_current			=	0;
            m_status			=	eGjkValid;
            m_shape				=	shapearg;
            m_distance			=	0;
            /* Initialize simplex		*/
            m_simplices[0].rank	=	0;
            m_ray				=	guess;
            double	sqrl=	m_ray.length2();
            appendvertice(m_simplices[0],sqrl>0?-m_ray:btVector3(1,0,0));
            m_simplices[0].p[0]	=	1;
            m_ray				=	m_simplices[0].c[0].w;
            sqdist				=	sqrl;
            lastw[0]			=
            lastw[1]			=
            lastw[2]			=
            lastw[3]			=	m_ray;
            /* Loop						*/
            do	{
                string 		next=1-m_current;
                sSimplex&	cs=m_simplices[m_current];
                sSimplex&	ns=m_simplices[next];
                /* Check zero							*/
                double	rl=m_ray.length();
                if(rl<GJK_MIN_DISTANCE)
                {/* Touching or inside				*/
                    m_status=eGjkInside;
                    break;
                }
                /* Append new vertice in -'v' direction	*/
                appendvertice(cs,-m_ray);
                ref btVector3	w=cs.c[cs.rank-1].w;
                bool				found=false;
                for(U i=0;i<4;++i)
                {
                    if((w-lastw[i]).length2()<GJK_DUPLICATED_EPS)
                    { found=true;break; }
                }
                if(found)
                {/* Return old simplex				*/
                    removevertice(m_simplices[m_current]);
                    break;
                }
                else
                {/* Update lastw					*/
                    lastw[clastw=(clastw+1)&3]=w;
                }
                /* Check for termination				*/
                double	omega=btDot(m_ray,w)/rl;
                alpha=btMax(omega,alpha);
                if(((rl-alpha)-(GJK_ACCURARY*rl))<=0)
                {/* Return old simplex				*/
                    removevertice(m_simplices[m_current]);
                    break;
                }
                /* Reduce simplex						*/
                double	weights[4];
                U			mask=0;
                switch(cs.rank)
                {
                    case	2:	sqdist=projectorigin(	cs.c[0].w,
                                                     cs.c[1].w,
                                                     weights,mask);break;
                    case	3:	sqdist=projectorigin(	cs.c[0].w,
                                                     cs.c[1].w,
                                                     cs.c[2].w,
                                                     weights,mask);break;
                    case	4:	sqdist=projectorigin(	cs.c[0].w,
                                                     cs.c[1].w,
                                                     cs.c[2].w,
                                                     cs.c[3].w,
                                                     weights,mask);break;
                }
                if(sqdist>=0)
                {/* Valid	*/
                    ns.rank		=	0;
                    m_ray		=	btVector3(0,0,0);
                    m_current	=	next;
                    for(U i=0,ni=cs.rank;i<ni;++i)
                    {
                        if(mask&(1<<i))
                        {
                            ns.c[ns.rank]		=	cs.c[i];
                            ns.p[ns.rank++]		=	weights[i];
                            m_ray				+=	cs.c[i].w*weights[i];
                        }
                        else
                        {
                            m_free[m_nfree++]	=	cs.c[i];
                        }
                    }
                    if(mask==15) m_status=eGjkInside;
                }
                else
                {/* Return old simplex				*/
                    removevertice(m_simplices[m_current]);
                    break;
                }
                m_status=((++iterations)<GJK_MAX_ITERATIONS)?m_status:eGjkFailed;
            } while(m_status==eGjkValid);
            m_simplex=&m_simplices[m_current];
            switch(m_status)
            {
                case	eGjkValid:		m_distance=m_ray.length();break;
                case	eGjkInside:	m_distance=0;break;
                default:
                {
                }
            }
            return(m_status);
        }
        bool					EncloseOrigin()
        {
            switch(m_simplex.rank)
            {
                case	1:
                {
                    for(U i=0;i<3;++i)
                    {
                        btVector3		axis=btVector3(0,0,0);
                        axis[i]=1;
                        appendvertice(*m_simplex, axis);
                        if(EncloseOrigin())	return(true);
                        removevertice(*m_simplex);
                        appendvertice(*m_simplex,-axis);
                        if(EncloseOrigin())	return(true);
                        removevertice(*m_simplex);
                    }
                }
                    break;
                case	2:
                {
                    btVector3	d=m_simplex.c[1].w-m_simplex.c[0].w;
                    for(U i=0;i<3;++i)
                    {
                        btVector3		axis=btVector3(0,0,0);
                        axis[i]=1;
                        btVector3	p=btCross(d,axis);
                        if(p.length2()>0)
                        {
                            appendvertice(*m_simplex, p);
                            if(EncloseOrigin())	return(true);
                            removevertice(*m_simplex);
                            appendvertice(*m_simplex,-p);
                            if(EncloseOrigin())	return(true);
                            removevertice(*m_simplex);
                        }
                    }
                }
                    break;
                case	3:
                {
                    btVector3	n=btCross(m_simplex.c[1].w-m_simplex.c[0].w,
                                              m_simplex.c[2].w-m_simplex.c[0].w);
                    if(n.length2()>0)
                    {
                        appendvertice(*m_simplex,n);
                        if(EncloseOrigin())	return(true);
                        removevertice(*m_simplex);
                        appendvertice(*m_simplex,-n);
                        if(EncloseOrigin())	return(true);
                        removevertice(*m_simplex);
                    }
                }
                    break;
                case	4:
                {
                    if(btFabs(det(	m_simplex.c[0].w-m_simplex.c[3].w,
                                  m_simplex.c[1].w-m_simplex.c[3].w,
                                  m_simplex.c[2].w-m_simplex.c[3].w))>0)
                        return(true);
                }
                    break;
            }
            return(false);
        }
        /* Internals	*/
        void				getsupport(ref btVector3 d,sSV& sv)
        {
            sv.d	=	d/d.length();
            sv.w	=	m_shape.Support(sv.d);
        }
        void				removevertice(sSimplex& simplex)
        {
            m_free[m_nfree++]=simplex.c[--simplex.rank];
        }
        void				appendvertice(sSimplex& simplex,ref btVector3 v)
        {
            simplex.p[simplex.rank]=0;
            simplex.c[simplex.rank]=m_free[--m_nfree];
            getsupport(v,*simplex.c[simplex.rank++]);
        }
        static double		det(ref btVector3 a,ref btVector3 b,ref btVector3 c)
        {
            return(	a.y*b.z*c.x+a.z*b.x*c.y-
                   a.x*b.z*c.y-a.y*b.x*c.z+
                   a.x*b.y*c.z-a.z*b.y*c.x);
        }
        static double		projectorigin(	ref btVector3 a,
                                          ref btVector3 b,
                                          double* w,U m)
        {
            btVector3	d=b-a;
            double	l=d.length2();
            if(l>GJK_SIMPLEX2_EPS)
            {
                double	t(l>0?-btDot(a,d)/l:0);
                if(t>=1)		{ w=0;w[1]=1;m=2;return(b.length2()); }
                else if(t<=0)	{ w[0]=1;w[1]=0;m=1;return(a.length2()); }
                else			{ w[0]=1-(w[1]=t);m=3;return((a+d*t).length2()); }
            }
            return(-1);
        }
        static double		projectorigin(	ref btVector3 a,
                                          ref btVector3 b,
                                          ref btVector3 c,
                                          double* w,U& m)
        {
            static string 		imd3[]={1,2,0};
            btVector3*	vt[]={&a,&b,&c};
            btVector3		dl[]={a-b,b-c,c-a};
            btVector3		n=btCross(dl[0],dl[1]);
            double		l=n.length2();
            if(l>GJK_SIMPLEX3_EPS)
            {
                double	mindist=-1;
                double	subw[2]={0,0};
                U			subm(0);
                for(U i=0;i<3;++i)
                {
                    if(btDot(*vt[i],btCross(dl[i],n))>0)
                    {
                        string 			j=imd3[i];
                        double	subd(projectorigin(*vt[i],*vt[j],subw,subm));
                        if((mindist<0)||(subd<mindist))
                        {
                            mindist		=	subd;
                            m			=	static_cast<U>(((subm&1)?1<<i:0)+((subm&2)?1<<j:0));
                            w[i]		=	subw[0];
                            w[j]		=	subw[1];
                            w[imd3[j]]	=	0;
                        }
                    }
                }
                if(mindist<0)
                {
                    double	d=btDot(a,n);
                    double	s=btSqrt(l);
                    btVector3	p=n*(d/l);
                    mindist	=	p.length2();
                    m		=	7;
                    w[0]	=	(btCross(dl[1],b-p)).length()/s;
                    w[1]	=	(btCross(dl[2],c-p)).length()/s;
                    w[2]	=	1-(w[0]+w[1]);
                }
                return(mindist);
            }
            return(-1);
        }
        static double		projectorigin(	ref btVector3 a,
                                          ref btVector3 b,
                                          ref btVector3 c,
                                          ref btVector3 d,
                                          double* w,U& m)
        {
            static string 		imd3[]={1,2,0};
            btVector3*	vt[]={&a,&b,&c,&d};
            btVector3		dl[]={a-d,b-d,c-d};
            double		vl=det(dl[0],dl[1],dl[2]);
            string ool			ng=(vl*btDot(a,btCross(b-c,a-b)))<=0;
            if(ng&&(btFabs(vl)>GJK_SIMPLEX4_EPS))
            {
                double	mindist=-1;
                double	subw[3]={0,0,0};
                U			subm(0);
                for(U i=0;i<3;++i)
                {
                    string 			j=imd3[i];
                    double	s=vl*btDot(d,btCross(dl[i],dl[j]));
                    if(s>0)
                    {
                        double	subd=projectorigin(*vt[i],*vt[j],d,subw,subm);
                        if((mindist<0)||(subd<mindist))
                        {
                            mindist		=	subd;
                            m			=	static_cast<U>((subm&1?1<<i:0)+
                                                           (subm&2?1<<j:0)+
                                                           (subm&4?8:0));
                            w[i]		=	subw[0];
                            w[j]		=	subw[1];
                            w[imd3[j]]	=	0;
                            w[3]		=	subw[2];
                        }
                    }
                }
                if(mindist<0)
                {
                    mindist	=	0;
                    m		=	15;
                    w[0]	=	det(c,b,d)/vl;
                    w[1]	=	det(a,c,d)/vl;
                    w[2]	=	det(b,a,d)/vl;
                    w[3]	=	1-(w[0]+w[1]+w[2]);
                }
                return(mindist);
            }
            return(-1);
        }
    };


enum	eEpaStatus
{
    eEpaValid,
    eEpaTouching,
    eEpaDegenerated,
    eEpaNonConvex,
    eEpaInvalidHull,
    eEpaOutOfFaces,
    eEpaOutOfVertices,
    eEpaAccuraryReached,
    eEpaFallBack,
    eEpaFailed
};


    // EPA
template <typename btConvexTemplate>
    struct	EPA
    {
        /* Types		*/
       
        struct	sFace
        {
            btVector3	n;
            double	d;
            typename GJK<btConvexTemplate>::sSV*		c[3];
            sFace*		f[3];
            sFace*		l[2];
            U1			e[3];
            U1			pass;
        };
        struct	sList
        {
            sFace*		root;
            U			count;
            sList() : root(0),count(0)	{}
        };
        struct	sHorizon
        {
            sFace*		cf;
            sFace*		ff;
            U			nf;
            sHorizon() : cf(0),ff(0),nf(0)	{}
        };
       
        /* Fields		*/
        eEpaStatus		m_status;
        typename GJK<btConvexTemplate>::sSimplex	m_result;
        btVector3		m_normal;
        double		m_depth;
        typename GJK<btConvexTemplate>::sSV				m_sv_store[EPA_MAX_VERTICES];
        sFace			m_fc_store[EPA_MAX_FACES];
        U				m_nextsv;
        sList			m_hull;
        sList			m_stock;
        /* Methods		*/
        EPA()
        {
            Initialize();
        }
        
        
        static inline void		bind(sFace* fa,U ea,sFace* fb,U eb)
        {
            fa.e[ea]=(U1)eb;fa.f[ea]=fb;
            fb.e[eb]=(U1)ea;fb.f[eb]=fa;
        }
        static inline void		append(sList list,sFace* face)
        {
            face.l	=	0;
            face.l[1]	=	list.root;
            if(list.root) list.root.l[0]=face;
            list.root	=	face;
            ++list.count;
        }
        static inline void		remove(sList& list,sFace* face)
        {
            if(face.l[1]) face.l[1].l[0]=face.l[0];
            if(face.l[0]) face.l[0].l[1]=face.l[1];
            if(face==list.root) list.root=face.l[1];
            --list.count;
        }
        
        
        void				Initialize()
        {
            m_status	=	eEpaFailed;
            m_normal	=	btVector3(0,0,0);
            m_depth		=	0;
            m_nextsv	=	0;
            for(U i=0;i<EPA_MAX_FACES;++i)
            {
                append(m_stock,&m_fc_store[EPA_MAX_FACES-i-1]);
            }
        }
        eEpaStatus			Evaluate(GJK<btConvexTemplate> gjk,ref btVector3 guess)
        {
            typename GJK<btConvexTemplate>::sSimplex&	simplex=*gjk.m_simplex;
            if((simplex.rank>1)&&gjk.EncloseOrigin())
            {
                
                /* Clean up				*/
                while(m_hull.root)
                {
                    sFace*	f = m_hull.root;
                    remove(m_hull,f);
                    append(m_stock,f);
                }
                m_status	=	eEpaValid;
                m_nextsv	=	0;
                /* Orient simplex		*/
                if(gjk.det(	simplex.c.w-simplex.c[3].w,
                           simplex.c[1].w-simplex.c[3].w,
                           simplex.c[2].w-simplex.c[3].w)<0)
                {
                    btSwap(simplex.c[0],simplex.c[1]);
                    btSwap(simplex.p[0],simplex.p[1]);
                }
                /* Build initial hull	*/
                sFace*	tetra[]={newface(simplex.c[0],simplex.c[1],simplex.c[2],true),
                    newface(simplex.c[1],simplex.c[0],simplex.c[3],true),
                    newface(simplex.c[2],simplex.c[1],simplex.c[3],true),
                    newface(simplex.c[0],simplex.c[2],simplex.c[3],true)};
                if(m_hull.count==4)
                {
                    sFace*		best=findbest();
                    sFace		outer=*best;
                    U			pass=0;
                    U			iterations=0;
                    bind(tetra[0],0,tetra[1],0);
                    bind(tetra[0],1,tetra[2],0);
                    bind(tetra[0],2,tetra[3],0);
                    bind(tetra[1],1,tetra[3],2);
                    bind(tetra[1],2,tetra[2],1);
                    bind(tetra[2],2,tetra[3],1);
                    m_status=eEpaValid;
                    for(;iterations<EPA_MAX_ITERATIONS;++iterations)
                    {
                        if(m_nextsv<EPA_MAX_VERTICES)
                        {
                            sHorizon		horizon;
                            typename GJK<btConvexTemplate>::sSV*			w=&m_sv_store[m_nextsv++];
                            bool			valid=true;
                            best.pass	=	(U1)(++pass);
                            gjk.getsupport(best.n,*w);
                            double	wdist=btDot(best.n,w.w)-best.d;
                            if(wdist>EPA_ACCURACY)
                            {
                                for(U j=0;(j<3)&&valid;++j)
                                {
                                    valid&=expand(	pass,w,
                                                  best.f[j],best.e[j],
                                                  horizon);
                                }
                                if(valid&(horizon.nf>=3))
                                {
                                    bind(horizon.cf,1,horizon.ff,2);
                                    remove(m_hull,best);
                                    append(m_stock,best);
                                    best=findbest();
                                    outer=*best;
                                } else { m_status=eEpaInvalidHull;break; }
                            } else { m_status=eEpaAccuraryReached;break; }
                        } else { m_status=eEpaOutOfVertices;break; }
                    }
                    btVector3	projection=outer.n*outer.d;
                    m_normal	=	outer.n;
                    m_depth		=	outer.d;
                    m_result.rank	=	3;
                    m_result.c	=	outer.c[0];
                    m_result.c[1]	=	outer.c[1];
                    m_result.c[2]	=	outer.c[2];
                    m_result.p[0]	=	btCross(	outer.c[1].w-projection,
                                                outer.c[2].w-projection).length();
                    m_result.p[1]	=	btCross(	outer.c[2].w-projection,
                                                outer.c[0].w-projection).length();
                    m_result.p[2]	=	btCross(	outer.c[0].w-projection,
                                                outer.c[1].w-projection).length();
                    double	sum=m_result.p[0]+m_result.p[1]+m_result.p[2];
                    m_result.p[0]	/=	sum;
                    m_result.p[1]	/=	sum;
                    m_result.p[2]	/=	sum;
                    return(m_status);
                }
            }
            /* Fallback		*/
            m_status	=	eEpaFallBack;
            m_normal	=	-guess;
            double	nl=m_normal.length();
            if(nl>0)
                m_normal	=	m_normal/nl;
            else
                m_normal	=	btVector3(1,0,0);
            m_depth	=	0;
            m_result.rank=1;
            m_result.c[0]=simplex.c[0];
            m_result.p[0]=1;
            return(m_status);
        }
        bool getedgedist(sFace* face, typename GJK<btConvexTemplate>::sSV* a, typename GJK<btConvexTemplate>::sSV* b, double dist)
        {
            btVector3 ba = b.w - a.w;
            btVector3 n_ab = btCross(ba, face.n); // Outward facing edge normal direction, on triangle plane
            double a_dot_nab = btDot(a.w, n_ab); // Only care about the sign to determine inside/outside, so not normalization required
            
            if(a_dot_nab < 0)
            {
                // Outside of edge a.b
                
                double ba_l2 = ba.length2();
                double a_dot_ba = btDot(a.w, ba);
                double b_dot_ba = btDot(b.w, ba);
                
                if(a_dot_ba > 0)
                {
                    // Pick distance vertex a
                    dist = a.w.length();
                }
                else if(b_dot_ba < 0)
                {
                    // Pick distance vertex b
                    dist = b.w.length();
                }
                else
                {
                    // Pick distance to edge a.b
                    double a_dot_b = btDot(a.w, b.w);
                    dist = btSqrt(btMax((a.w.length2() * b.w.length2() - a_dot_b * a_dot_b) / ba_l2, (double)0));
                }
                
                return true;
            }
            
            return false;
        }
        sFace*				newface(typename GJK<btConvexTemplate>::sSV* a,typename GJK<btConvexTemplate>::sSV* b,typename GJK<btConvexTemplate>::sSV* c,bool forced)
        {
            if(m_stock.root)
            {
                sFace*	face=m_stock.root;
                remove(m_stock,face);
                append(m_hull,face);
                face.pass	=	0;
                face.c[0]	=	a;
                face.c[1]	=	b;
                face.c[2]	=	c;
                face.n		=	btCross(b.w-a.w,c.w-a.w);
                double	l=face.n.length();
                string ool		v=l>EPA_ACCURACY;
                
                if(v)
                {
                    if(!(getedgedist(face, a, b, face.d) ||
                         getedgedist(face, b, c, face.d) ||
                         getedgedist(face, c, a, face.d)))
                    {
                        // Origin projects to the interior of the triangle
                        // Use distance to triangle plane
                        face.d = btDot(a.w, face.n) / l;
                    }
                    
                    face.n /= l;
                    if(forced || (face.d >= -EPA_PLANE_EPS))
                    {
                        return face;
                    }
                    else
                        m_status=eEpaNonConvex;
                }
                else
                    m_status=eEpaDegenerated;
                
                remove(m_hull, face);
                append(m_stock, face);
                return 0;
                
            }
            m_status = m_stock.root ? eEpaOutOfVertices : eEpaOutOfFaces;
            return 0;
        }
        sFace*				findbest()
        {
            sFace*		minf=m_hull.root;
            double	mind=minf.d*minf.d;
            for(sFace* f=minf.l[1];f;f=f.l[1])
            {
                double	sqd=f.d*f.d;
                if(sqd<mind)
                {
                    minf=f;
                    mind=sqd;
                }
            }
            return(minf);
        }
        bool				expand(U pass,typename GJK<btConvexTemplate>::sSV* w,sFace* f,U e,sHorizon& horizon)
        {
            static string 	i1m3[]={1,2,0};
            static string 	i2m3[]={2,0,1};
            if(f.pass!=pass)
            {
                string 	e1=i1m3[e];
                if((btDot(f.n,w.w)-f.d)<-EPA_PLANE_EPS)
                {
                    sFace*	nf=newface(f.c[e1],f.c[e],w,false);
                    if(nf)
                    {
                        bind(nf,0,f,e);
                        if(horizon.cf) bind(horizon.cf,1,nf,2); else horizon.ff=nf;
                        horizon.cf=nf;
                        ++horizon.nf;
                        return(true);
                    }
                }
                else
                {
                    string 	e2=i2m3[e];
                    f.pass		=	(U1)pass;
                    if(	expand(pass,w,f.f[e1],f.e[e1],horizon)&&
                       expand(pass,w,f.f[e2],f.e[e2],horizon))
                    {
                        remove(m_hull,f);
                        append(m_stock,f);
                        return(true);
                    }
                }
            }
            return(false);
        }
        
    };
    
    template <typename btConvexTemplate>
    static void	Initialize(	btConvexTemplate a, btConvexTemplate& b,
                           btGjkEpaSolver3::sResults& results,
                           MinkowskiDiff<btConvexTemplate>& shape)
    {
        /* Results		*/ 
        results.witnesses	=
        results.witnesses[1]	=	btVector3(0,0,0);
        results.status			=	btGjkEpaSolver3::sResults::Separated;
        /* Shape		*/ 
       
        shape.m_toshape1		=	b.getWorldTransform().getBasis().transposeTimes(a.getWorldTransform().getBasis());
        shape.m_toshape0		=	a.getWorldTransform().inverseTimes(b.getWorldTransform());
        
    }
    

//
// Api
//



//
template <typename btConvexTemplate>
bool		btGjkEpaSolver3_Distance(btConvexTemplate& a, btConvexTemplate& b,
                                      ref btVector3 guess,
                                      btGjkEpaSolver3::sResults& results)
{
    MinkowskiDiff<btConvexTemplate>			shape(a,b);
    Initialize(a,b,results,shape);
    GJK<btConvexTemplate>				gjk(a,b);
    eGjkStatus	gjk_status=gjk.Evaluate(shape,guess);
    if(gjk_status==eGjkValid)
    {
        btVector3	w0=btVector3(0,0,0);
        btVector3	w1=btVector3(0,0,0);
        for(U i=0;i<gjk.m_simplex.rank;++i)
        {
            double	p=gjk.m_simplex.p[i];
            w0+=shape.Support( gjk.m_simplex.c[i].d,0)*p;
            w1+=shape.Support(-gjk.m_simplex.c[i].d,1)*p;
        }
        results.witnesses[0]	=	a.getWorldTransform()*w0;
        results.witnesses[1]	=	a.getWorldTransform()*w1;
        results.normal			=	w0-w1;
        results.distance		=	results.normal.length();
        results.normal			/=	results.distance>GJK_MIN_DISTANCE?results.distance:1;
        return(true);
    }
    else
    {
        results.status	=	gjk_status==eGjkInside?
        btGjkEpaSolver3::sResults::Penetrating	:
        btGjkEpaSolver3::sResults::GJK_Failed	;
        return(false);
    }
}


template <typename btConvexTemplate>
bool	btGjkEpaSolver3_Penetration(btConvexTemplate& a,
                                     btConvexTemplate& b,
                                     ref btVector3 guess,
                                     btGjkEpaSolver3::sResults& results)
{
    MinkowskiDiff<btConvexTemplate>			shape(a,b);
    Initialize(a,b,results,shape);
    GJK<btConvexTemplate>				gjk(a,b);
    eGjkStatus	gjk_status=gjk.Evaluate(shape,-guess);
    switch(gjk_status)
    {
        case	eGjkInside:
        {
            EPA<btConvexTemplate>				epa;
            eEpaStatus	epa_status=epa.Evaluate(gjk,-guess);
            if(epa_status!=eEpaFailed)
            {
                btVector3	w0=btVector3(0,0,0);
                for(U i=0;i<epa.m_result.rank;++i)
                {
                    w0+=shape.Support(epa.m_result.c[i].d,0)*epa.m_result.p[i];
                }
                results.status			=	btGjkEpaSolver3::sResults::Penetrating;
                results.witnesses[0]	=	a.getWorldTransform()*w0;
                results.witnesses[1]	=	a.getWorldTransform()*(w0-epa.m_normal*epa.m_depth);
                results.normal			=	-epa.m_normal;
                results.distance		=	-epa.m_depth;
                return(true);
            } else results.status=btGjkEpaSolver3::sResults::EPA_Failed;
        }
            break;
        case	eGjkFailed:
            results.status=btGjkEpaSolver3::sResults::GJK_Failed;
            break;
        default:
        {
        }
    }
    return(false);
}

#if 0
int	btComputeGjkEpaPenetration2(btCollisionDescription& colDesc, btDistanceInfo* distInfo)
{
    btGjkEpaSolver3::sResults results;
    btVector3 guess = colDesc.m_firstDir;
    
    bool res = btGjkEpaSolver3::Penetration(colDesc.m_objA,colDesc.m_objB,
                                            colDesc.m_transformA,colDesc.m_transformB,
                                            colDesc.m_localSupportFuncA,colDesc.m_localSupportFuncB,
                                            guess,
                                            results);
    if (res)
    {
        if ((results.status==btGjkEpaSolver3::sResults::Penetrating) || results.status==GJK::eStatus::Inside)
        {
            //normal could be 'swapped'
            
            distInfo.m_distance = results.distance;
            distInfo.m_normalBtoA = results.normal;
            btVector3 tmpNormalInB = results.witnesses[1]-results.witnesses[0];
            double lenSqr = tmpNormalInB.length2();
            if (lenSqr <= (SIMD_EPSILON*SIMD_EPSILON))
            {
                tmpNormalInB = results.normal;
                lenSqr = results.normal.length2();
            }
            
            if (lenSqr > (SIMD_EPSILON*SIMD_EPSILON))
            {
                tmpNormalInB /= btSqrt(lenSqr);
                double distance2 = -(results.witnesses[0]-results.witnesses[1]).length();
                //only replace valid penetrations when the result is deeper (check)
                //if ((distance2 < results.distance))
                {
                    distInfo.m_distance = distance2;
                    distInfo.m_pointOnA= results.witnesses[0];
                    distInfo.m_pointOnB= results.witnesses[1];
                    distInfo.m_normalBtoA= tmpNormalInB;
                    return 0;
                }
            }
        }
        
    }
    
    return -1;
}
#endif

template <typename btConvexTemplate, typename btDistanceInfoTemplate>
int	btComputeGjkDistance(btConvexTemplate a, btConvexTemplate& b,
                         btGjkCollisionDescription& colDesc, btDistanceInfoTemplate* distInfo)
{
    btGjkEpaSolver3::sResults results;
    btVector3 guess = colDesc.m_firstDir;
    
    bool isSeparated = btGjkEpaSolver3_Distance(	a,b,
                                                 guess,
                                                 results);
    if (isSeparated)
    {
        distInfo.m_distance = results.distance;
        distInfo.m_pointOnA= results.witnesses;
        distInfo.m_pointOnB= results.witnesses[1];
        distInfo.m_normalBtoA= results.normal;
        return 0;
    }
    
    return -1;
}

/* Symbols cleanup		*/ 

#undef GJK_MAX_ITERATIONS
#undef GJK_ACCURARY
#undef GJK_MIN_DISTANCE
#undef GJK_DUPLICATED_EPS
#undef GJK_SIMPLEX2_EPS
#undef GJK_SIMPLEX3_EPS
#undef GJK_SIMPLEX4_EPS

#undef EPA_MAX_VERTICES
#undef EPA_MAX_FACES
#undef EPA_MAX_ITERATIONS
#undef EPA_ACCURACY
#undef EPA_FALLBACK
#undef EPA_PLANE_EPS
#undef EPA_INSIDE_EPS



#endif //BT_GJK_EPA3_H

