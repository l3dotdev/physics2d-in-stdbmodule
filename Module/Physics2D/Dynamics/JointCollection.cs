using System.Collections;
using StdbModule.Physics2D.Dynamics.Joints;

namespace StdbModule.Physics2D.Dynamics;

public class JointCollection : IEnumerable<Joint>, ICollection<Joint>, IList<Joint>
{
    internal readonly List<Joint> _list = new(32);
    internal int _generationStamp = 0;

    public JointCollection() { }

    public JointEnumerator GetEnumerator()
    {
        return new JointEnumerator(this, _list);
    }


    #region IList<Joint>
    public Joint this[int index]
    {
        get { return _list[index]; }
        set { throw new NotSupportedException(); }
    }

    public int IndexOf(Joint item)
    {
        return _list.IndexOf(item);
    }

    void IList<Joint>.Insert(int index, Joint item)
    {
        throw new NotSupportedException();
    }

    void IList<Joint>.RemoveAt(int index)
    {
        throw new NotSupportedException();
    }
    #endregion IList<Joint>


    #region ICollection<Joint>

    public bool IsReadOnly { get { return true; } }

    public int Count { get { return _list.Count; } }

    void ICollection<Joint>.Add(Joint item)
    {
        throw new NotSupportedException();
    }

    bool ICollection<Joint>.Remove(Joint item)
    {
        throw new NotSupportedException();
    }

    void ICollection<Joint>.Clear()
    {
        throw new NotSupportedException();
    }

    public bool Contains(Joint item)
    {
        return _list.Contains(item);
    }

    public void CopyTo(Joint[] array, int arrayIndex)
    {
        _list.CopyTo(array, arrayIndex);
    }

    #endregion ICollection<Joint>


    #region IEnumerable<Joint>
    IEnumerator<Joint> IEnumerable<Joint>.GetEnumerator()
    {
        return new JointEnumerator(this, _list);
    }
    #endregion IEnumerable<Joint>


    #region IEnumerable
    IEnumerator IEnumerable.GetEnumerator()
    {
        return new JointEnumerator(this, _list);
    }

    #endregion IEnumerable


    public struct JointEnumerator(JointCollection collection, List<Joint> list) : IEnumerator<Joint>
    {
        private JointCollection _collection = collection;
        private List<Joint> _list = list;
        private readonly int _generationStamp = collection._generationStamp;
        int i = -1;

        public readonly Joint Current
        {
            get
            {
                if (_generationStamp == _collection._generationStamp)
                    return _list[i];
                else
                    throw new InvalidOperationException("Collection was modified.");
            }
        }

        #region IEnumerator<Joint>
        readonly Joint IEnumerator<Joint>.Current
        {
            get
            {
                if (_generationStamp == _collection._generationStamp)
                    return _list[i];
                else
                    throw new InvalidOperationException("Collection was modified.");
            }
        }
        #endregion IEnumerator<Joint>

        #region IEnumerator
        public bool MoveNext()
        {
            if (_generationStamp != _collection._generationStamp)
                throw new InvalidOperationException("Collection was modified.");

            return ++i < _list.Count;
        }


        readonly object IEnumerator.Current
        {
            get
            {
                if (_generationStamp == _collection._generationStamp)
                    return _list[i];
                else
                    throw new InvalidOperationException();
            }
        }

        void IDisposable.Dispose()
        {
            _collection = null;
            _list = null;
            i = -1;
        }

        void IEnumerator.Reset()
        {
            i = -1;
        }
        #endregion IEnumerator
    }
}