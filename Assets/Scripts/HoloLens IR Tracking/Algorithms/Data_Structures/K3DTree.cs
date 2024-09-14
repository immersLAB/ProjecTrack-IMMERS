using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Spatial.Euclidean;
public class K3DTree
{

    private int CompareByDimX(Vector3D a, Vector3D b)
    {
        if (a.X < b.X) return -1;
        if (a.X == b.X) return 0;
        return 1;
    }
    private int CompareByDimY(Vector3D a, Vector3D b)
    {
        if (a.Y < b.Y) return -1;
        if (a.Y == b.Y) return 0;
        return 1;
    }
    private int CompareByDimZ(Vector3D a, Vector3D b)
    {
        if (a.Z < b.Z) return -1;
        if (a.Z == b.Z) return 0;
        return 1;    }

    //private int CompareByDim(Vector3D a, Vector3D b, int d)
    //{
    //    if (a.Value(d) < b.Value(d)) return -1;
    //    if (a.Value(d) == b.Value(d)) return 0;
    //    return 1;
    //}

    class TreeNode
    {
        public Vector3D data;
        public TreeNode left, right;
        public TreeNode(Vector3D p)
        {
            data = p;
        }
    }

    List<Vector3D> points = new List<Vector3D>();
    List<TreeNode> indexes = new List<TreeNode>();
    private TreeNode root;
    public K3DTree(List<Vector3D> p)
    {
        points = p;
        
        GenerateTree();
    }
    private void GenerateTree()
    {
        //int[] pX = new int[points.Count];// List<int>();
        //int[] pY = new int[points.Count];
        //int[] pZ = new int[points.Count];

        //for (int i = 0; i < points.Count; i++)
        //{
        //    pX[i] = i;
        //    pY[i] = i;
        //    pZ[i] = i;
        //}

        if (points.Count == 1)
        {
            root = new TreeNode(points[0]);
            return;
        }
        if (points.Count == 0)
        {
            return;
        }

        Stack<TreeNode> nodeStack = new Stack<TreeNode>();
        Stack<List<Vector3D>> listStack = new Stack<List<Vector3D>>();
        listStack.Push(points);

        do
        {
            List<Vector3D> currentPoints = listStack.Pop();

            // TODO cut down on these if-else's
            if (currentPoints.Count == 0)
            {
                if (nodeStack.Peek().right == null)
                {
                    continue;
                }
                if (nodeStack.Peek().left == null)
                {
                    nodeStack.Pop();
                    continue;
                }
            }

            if (currentPoints.Count == 1)
            {
                if (nodeStack.Peek().right == null)
                {
                    nodeStack.Peek().right = new TreeNode(currentPoints[0]);
                }
                else if (nodeStack.Peek().left == null)
                {
                    //recentNode = nodeStack.Pop();
                    nodeStack.Pop().left = new TreeNode(currentPoints[0]);
                }
                continue;
            }

            if (nodeStack.Count % 3 == 0)
            {
                currentPoints.Sort(CompareByDimX);
            }
            else if (nodeStack.Count % 3 == 1)
            {
                currentPoints.Sort(CompareByDimY);
            }
            else
            {
                currentPoints.Sort(CompareByDimZ);
            }

            int idx = (currentPoints.Count-1)/2;
            TreeNode insertedNode = new TreeNode(currentPoints[idx]);
            if (nodeStack.Count == 0)
            {
                root = insertedNode;
            }
            else
            {
                if (nodeStack.Peek().right == null)
                {
                    nodeStack.Peek().right = insertedNode;
                }
                else if (nodeStack.Peek().left == null)
                {
                    nodeStack.Peek().left = insertedNode;
                }
            }
            nodeStack.Push(insertedNode);
            listStack.Push(currentPoints.GetRange(0, idx));
            if (currentPoints.Count % 2 == 1)
            {
                listStack.Push(currentPoints.GetRange(idx + 1, idx));
            }
            else
            {
                listStack.Push(currentPoints.GetRange(idx + 1, idx+1));
            }
        } while (listStack.Count != 0);
    }

    public bool Empty()
    {
        return root == null;
    }
    public Vector3D FindClosest(Vector3D p)
    {
        TreeNode bestNode = FindClosest(root, p, 0);
        if (bestNode == null)
            return Vector3D.NaN;
        return bestNode.data;
    }

    // TODO: Clean so no reliance on null checks; non-recursive implementation
    private TreeNode FindClosest(TreeNode node, Vector3D target, int depth)
    {
        if (node == null)
            return null;

        if (node.left == null && node.right == null)
        {
            return node;
        }

        int dim = depth % 3;
        TreeNode rejectedNode, traversedNode;
        if ((dim == 0 && target.X <= node.data.X) || 
            (dim == 1 && target.Y <= node.data.Y) || 
            (dim == 2 && target.Z <= node.data.Z))
        {
            rejectedNode = node.right;
            traversedNode = node.left;
        }
        else
        {
            rejectedNode = node.left;
            traversedNode = node.right;
        }
        

        TreeNode bestNode = FindClosest(traversedNode, target, depth+1);
        
        if (bestNode == null)
        {
            bestNode = node;
        }

        double bestNodeDistSq = (bestNode.data - target).DotProduct(bestNode.data - target);
        double nodeDistSq = (node.data - target).DotProduct(node.data - target);
        // Replace best node if current node is better
        if (bestNodeDistSq > nodeDistSq)
        {
            bestNode = node;
            bestNodeDistSq = nodeDistSq;
        }

        // Check if there may be a better node on the other hyperplane
        if ((dim == 0 && bestNodeDistSq > (node.data.X - target.X) * (node.data.X - target.X)) ||
            (dim == 1 && bestNodeDistSq > (node.data.Y - target.Y) * (node.data.Y - target.Y)) ||
            (dim == 2 && bestNodeDistSq > (node.data.Z - target.Z) * (node.data.Z - target.Z)))
        {
            TreeNode candidateNode = FindClosest(rejectedNode, target, depth + 1);
            if (candidateNode != null && (candidateNode.data - target).DotProduct(candidateNode.data - target) < bestNodeDistSq)
                return candidateNode;
        }
        return bestNode;
    }

}
