using System;
using System.Collections.Generic;
using System.Diagnostics;
using MathNet.Spatial.Euclidean;
using MathNet.Numerics.LinearAlgebra;
// GoICP algorithm adapted from paper
struct BnBRotNode : IComparable<BnBRotNode>
{
	public double a, b, c;
	public double om; // rotation angle
	public double lb, ub;
	public int l;
	public int CompareTo(BnBRotNode b)
	{
		if (lb == b.lb)
		{
			return 0;
		}
		if (lb < b.lb)
		{
			return -1;
		}
		return 1;
	}

	public static bool operator <(BnBRotNode a, BnBRotNode b)
	{
		return (a.lb != b.lb) ? a.lb > b.lb : a.om < b.om;
	}

	public static bool operator <=(BnBRotNode a, BnBRotNode b)
	{
		return (a.lb != b.lb) ? a.lb >= b.lb : a.om <= b.om;
	}

	public static bool operator >(BnBRotNode a, BnBRotNode b)
	{
		return (a.lb != b.lb) ? a.lb < b.lb : a.om > b.om;
	}

	public static bool operator >=(BnBRotNode a, BnBRotNode b)
	{
		return (a.lb != b.lb) ? a.lb <= b.lb : a.om >= b.om;
	}
}

struct BnBTranslateNode : IComparable<BnBTranslateNode>
{
	public double x,y,z;
	public double hw; // cube halfwidth
	public double lb, ub;

	public static bool operator <(BnBTranslateNode a, BnBTranslateNode b)
	{
		return (a.lb != b.lb) ? a.lb > b.lb : a.hw < b.hw;
	}

	public static bool operator >(BnBTranslateNode a, BnBTranslateNode b)
	{
		return (a.lb != b.lb) ? a.lb < b.lb : a.hw > b.hw;
	}
	public int CompareTo(BnBTranslateNode b)
	{
		if (lb == b.lb)
		{
			return 0;
		}
		if (lb < b.lb)
		{
			return -1;
		}
		return 1;
	}
}
class GoICPSolver
{
	const float SSEThresh = 0.001f;
	//private float _errorSq;
	//public float ErrorSq
	//{
	//	get
	//	{
	//		return _errorSq;
	//	}
	//}

	const int MAXROTLEVEL = 15;
	const float SQRT3 = 1.732050808f;

	public static RegistrationData Register(PointCloud src, PointCloud dst, int iter = 100)
	{
		//Initialize(src, dst);
		K3DTree tree = new K3DTree(dst.GetPointsAsList());
		return OuterBnB(src, dst, tree, iter);
	}

	public static RegistrationData Register(PointCloud src, PointCloud dst, K3DTree tree, int iter = 100)
	{
		//Initialize(src, dst);
		return OuterBnB(src, dst, tree, iter);
	}

	// Outer branch and bound
    private static RegistrationData OuterBnB(PointCloud src, PointCloud dst, K3DTree tree , int iter = 100)
    {
        int i, j;

		// nodeRot: intermediate node 
		// nodeRotParent: node that is pushed off quue
		// optRNode: Node with optimal rotation
        BnBRotNode nodeRot = new BnBRotNode(), nodeRotParent, optRNode;

		// nodeTrans: intermediate node 
		// optTNode: node with optimal translation
		BnBTranslateNode nodeTrans = new BnBTranslateNode(), optTNode;
        double v1, v2, v3, t;
        PriorityQueue<BnBRotNode> queueRot = new PriorityQueue<BnBRotNode>();
        double[] minDis = new double[src.Count];

        List<Vector3D> sPts = src.GetPointsAsList();

        double ub = 0, lb = 0;
		RegistrationData optData = new RegistrationData();

		optData.R = Matrix<double>.Build.DenseIdentity(3);
		optData.t = new Vector3D();

		Quaternion optR = Quaternion.One;
		Vector3D optT = new Vector3D();

        BnBRotNode initNodeRot = new BnBRotNode();

        initNodeRot.a = -MathF.PI;
        initNodeRot.b = -MathF.PI;
        initNodeRot.c = -MathF.PI;
        initNodeRot.om = 2*MathF.PI;
        initNodeRot.l = 0;

		double[][] maxRotDist = new double[MAXROTLEVEL][];

		for (j = 0; j < MAXROTLEVEL; j++)
		{
			maxRotDist[j] = new double[sPts.Count];
			double sigma = initNodeRot.om / MathF.Pow(2.0f, j) / 2; // Half-side length of each level of rotation subcube
			double maxAngle = SQRT3 * sigma;

			if (maxAngle > MathF.PI)
				maxAngle = MathF.PI;

			for (i = 0; i < sPts.Count; i++)
			{
				if (j == 0)
				{
					Vector3D closestPt = tree.FindClosest(sPts[i]);
					minDis[i] = (sPts[i]-closestPt).DotProduct(sPts[i] - closestPt);
					optData.errorSq += minDis[i];
					minDis[i] = Math.Sqrt(minDis[i]);
				}
				maxRotDist[j][i] = 2*Math.Sin(maxAngle/2) * sPts[i].Length;
			}
		}
	

		RegistrationData initReg = ICPSolver.Register(src, dst, tree);
		if (initReg.errorSq < optData.errorSq)
		{
			optData.errorSq = initReg.errorSq;
			optData.R = initReg.R;
			optData.t = initReg.t;
		}

		int num_iterations = 0;
		queueRot.Enqueue(initNodeRot);
        while (!queueRot.Empty() && num_iterations++ < iter)
        {
            nodeRotParent = queueRot.Dequeue();
            if ((optData.errorSq - nodeRotParent.lb) <= SSEThresh)
            {
                break;
            }

            nodeRot.om = nodeRotParent.om / 2;
            nodeRot.l = nodeRotParent.l + 1;

			Matrix<double> loopR = Matrix<double>.Build.DenseIdentity(3);
            for (j = 0; j < 8; j++)
            {
				PointCloud sv = new PointCloud(src.GetPointsAsList());

				nodeRot.a = nodeRotParent.a + (j & 1) * nodeRot.om;
                nodeRot.b = nodeRotParent.b + ((j >> 1) & 1) * nodeRot.om;
                nodeRot.c = nodeRotParent.c + ((j >> 2) & 1) * nodeRot.om;

                v1 = nodeRot.a + nodeRot.om / 2;
                v2 = nodeRot.b + nodeRot.om / 2;
                v3 = nodeRot.c + nodeRot.om / 2;

				t = Math.Sqrt(v1 * v1 + v2 * v2 + v3 * v3);

				if (t - SQRT3 * nodeRot.om / 2 > Math.PI)
                {
                    continue;
                }

                if (t > 0)
                {
					
                    double c = Math.Cos(t);
                    double s = Math.Sin(t);
					Vector3D e = new Vector3D(v1 / t, v2 / t, v3 / t);
					UnitVector3D uv = UnitVector3D.Create(e.X, e.Y, e.Z);
					MathNet.Spatial.Units.Angle ang = MathNet.Spatial.Units.Angle.FromRadians(t);
					loopR = Matrix3D.RotationAroundArbitraryVector(uv, ang);

                    for (i = 0; i < sv.Count; i++)
                    {
						Vector3D v = sv.GetPoint(i);
						sv.SetPoint(c * v + s * e.CrossProduct(v) + (1 - c) * (e.DotProduct(v)) * e, i);
                    }
                }

                ub = InnerBnB(null, false, ref nodeTrans, optData.errorSq, sv, dst, tree);

                if (ub < optData.errorSq)
                {
                    optData.errorSq = ub;

                    optRNode = nodeRot;
                    optTNode = nodeTrans;

					optData.R = loopR;
                    optData.t = new Vector3D(optTNode.x, optTNode.y, optTNode.z) + optTNode.hw / 2 * new Vector3D(1,1,1);

                    RegistrationData result = ICPSolver.Register(src, dst, optData.R, optData.t, tree);
                    if (result.errorSq < optData.errorSq)
                    {
						optData.errorSq = result.errorSq;
						optData.R = result.R;
						optData.t = result.t;
					}

					PriorityQueue<BnBRotNode> queueRotNew = new PriorityQueue<BnBRotNode>();
                    while (!queueRot.Empty())
                    {
                        BnBRotNode node = queueRot.Dequeue();
                        if (node.lb < optData.errorSq)
                        {
                            queueRotNew.Enqueue(node);
                        }
                        else
                        {
                            break;
                        }
                    }
                    queueRot = queueRotNew;
                }
				
                lb = InnerBnB(maxRotDist[nodeRot.l], true, ref nodeTrans, optData.errorSq, sv, dst, tree);
                if (lb >= optData.errorSq)
                {
                    continue;
                }
                nodeRot.ub = ub;
                nodeRot.lb = lb;
                queueRot.Enqueue(nodeRot);
            }
        }
		//optData.errorSq = optError;
		//optData.T = new Pose(optT, optR).ToMatrix();
		return optData;
	}


	// Inner Branch-and-Bound, iterating over the translation space
	private static double InnerBnB(double[] maxRotDisL, bool translateEmpty, ref BnBTranslateNode nodeTransOut, double optError, PointCloud src, PointCloud dst, K3DTree tree)
	{
		int i, j;
		double transX, transY, transZ;
		double lb, ub, optErrorT;
		double dis, maxTransDis;
		double[] minDis = new double[src.Count];
		BnBTranslateNode nodeTrans = new BnBTranslateNode(), nodeTransParent;
		PriorityQueue<BnBTranslateNode> queueTrans = new PriorityQueue<BnBTranslateNode>();

		// Set optimal translation error to overall so-far optimal error
		// Investigating translation nodes that are sub-optimal overall is redundant
		optErrorT = optError;

		BnBTranslateNode initNodeTrans = new BnBTranslateNode();
		initNodeTrans.x = -0.5f;
		initNodeTrans.y = -0.5f;
		initNodeTrans.z = -0.5f;
		initNodeTrans.hw = 1f;
		initNodeTrans.lb = 0;
		// Push top-level translation node into the priority queue
		queueTrans.Enqueue(initNodeTrans);

		//
		while (!queueTrans.Empty())
		{

			nodeTransParent = queueTrans.Dequeue();

			if (optErrorT - nodeTransParent.lb < SSEThresh)
			{
				break;
			}

			nodeTrans.hw = nodeTransParent.hw / 2;
			maxTransDis = SQRT3 / 2.0f * nodeTrans.hw;

			for (j = 0; j < 8; j++)
			{
				nodeTrans.x = nodeTransParent.x + (j & 1) * nodeTrans.hw;
				nodeTrans.y = nodeTransParent.y + (j >> 1 & 1) * nodeTrans.hw;
				nodeTrans.z = nodeTransParent.z + (j >> 2 & 1) * nodeTrans.hw;

				transX = nodeTrans.x + nodeTrans.hw / 2;
				transY = nodeTrans.y + nodeTrans.hw / 2;
				transZ = nodeTrans.z + nodeTrans.hw / 2;

				// For each data point, calculate the distance to it's closest point in the model cloud
				for (i = 0; i < src.Count; i++)
                {
                    // Find distance between transformed point and closest point in model set ||R_r0 * x + t0 - y||
                    // pDataTemp is the data points rotated by R0
                    Vector3D p = src.GetPoint(i);
                    p = p + new Vector3D(transX, transY, transZ);

					Vector3D closestPt = tree.FindClosest(p);
					minDis[i] = (closestPt - p).Length;

					// Subtract the rotation uncertainty radius if calculating the rotation lower bound
					// maxRotDisL == NULL when calculating the rotation upper bound
					if (maxRotDisL != null)
						minDis[i] -= maxRotDisL[i];

					if (minDis[i] < 0)
					{
						minDis[i] = 0;
					}
				}

				// For each data point, find the incremental upper and lower bounds
				ub = 0;
				lb = 0;
				for (i = 0; i < src.Count; i++)
				{
					ub += minDis[i] * minDis[i];
					dis = minDis[i] - maxTransDis;
					if (dis > 0)
						lb += dis * dis;

				}

				// If upper bound is better than best, update optErrorT and optTransOut (optimal translation node)
				if (ub < optErrorT)
				{
					optErrorT = ub;
					if (!translateEmpty)
						nodeTransOut = nodeTrans;
				}

				// Remove subcube from queue if lb is bigger than optErrorT
				if (lb >= optErrorT)
				{
					//discard
					continue;
				}

				nodeTrans.ub = ub;
				nodeTrans.lb = lb;
				queueTrans.Enqueue(nodeTrans);
			}
		}

		return optErrorT;
	}

}
