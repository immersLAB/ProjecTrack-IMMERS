using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

class PriorityQueue<T> where T : IComparable<T>
{
    List<T> H = new List<T>();
    // Function to insert a
    // new element in
    // the Binary Heap
    int size = -1;

    public int Count 
    {
        get {
            return H.Count;
                }
    }
    public void Enqueue(T p)
    {
        size = size + 1;
        H.Add(p);
        // Shift Up to maintain
        // heap property
        shiftUp(size);
    }

    // Function to get value of
    // the current maximum element
    public T Peek()
    {
        return H[0];
    }

    public bool Empty()
    {
        return H.Count == 0;
    }
    public T Dequeue()
    {
        if (H.Count == 0)
            throw new InvalidOperationException("Unable to Dequeue - Queue Empty.");

        T result = H[0];

        // Replace the value
        // at the root with
        // the last leaf
        H[0] = H[size];
        H.RemoveAt(size);
        size = size - 1;

        // Shift down the replaced
        // element to maintain the
        // heap property
        shiftDown(0);
        return result;
    }

    // Function to return the index of the
    // parent node of a given node
    private int parent(int i)
    {
        return (i - 1) / 2;
    }

    // Function to return the index of the
    // left child of the given node
    private int leftChild(int i)
    {
        return ((2 * i) + 1);
    }

    // Function to return the index of the
    // right child of the given node
    private int rightChild(int i)
    {
        return ((2 * i) + 2);
    }

    // Function to shift up the
    // node in order to maintain
    // the heap property
    private void shiftUp(int i)
    {
        while (i > 0 &&
               H[parent(i)].CompareTo(H[i]) > 0)
        {

            // Swap parent and current node
            Swap(parent(i), i);

            // Update i to parent of i
            i = parent(i);
        }
    }

    // Function to shift down the node in
    // order to maintain the heap property
    private void shiftDown(int i)
    {
        int maxIndex = i;

        // Left Child
        int l = leftChild(i);

        if (l <= size &&
            H[l].CompareTo(H[maxIndex]) < 0)
        {
            maxIndex = l;
        }

        // Right Child
        int r = rightChild(i);

        if (r <= size &&
            H[r].CompareTo(H[maxIndex]) < 0)
        {
            maxIndex = r;
        }

        // If i not same as maxIndex
        if (i != maxIndex)
        {
            Swap(i, maxIndex);
            shiftDown(maxIndex);
        }
    }

    private void Swap(int i, int j)
    {
        T temp = H[i];
        H[i] = H[j];
        H[j] = temp;
    }


}