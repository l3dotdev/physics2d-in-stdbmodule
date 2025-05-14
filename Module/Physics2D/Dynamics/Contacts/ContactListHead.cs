using System.Collections;

namespace StdbModule.Physics2D.Dynamics.Contacts;

/// <summary>
/// Head of a circular doubly linked list.
/// </summary>
public class ContactListHead : Contact, IEnumerable<Contact>
{
    internal ContactListHead() : base(null, 0, null, 0)
    {
        Prev = this;
        Next = this;
    }

    IEnumerator<Contact> IEnumerable<Contact>.GetEnumerator()
    {
        return new ContactEnumerator(this);
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return new ContactEnumerator(this);
    }


    #region Nested type: ContactEnumerator

    private struct ContactEnumerator : IEnumerator<Contact>
    {
        private ContactListHead _head;
        private Contact _current;

        public readonly Contact Current => _current;
        readonly object IEnumerator.Current => _current;


        public ContactEnumerator(ContactListHead contact)
        {
            _head = contact;
            _current = _head;
        }

        public void Reset()
        {
            _current = _head;
        }

        public bool MoveNext()
        {
            if (_current.Next is null)
            {
                return false;
            }

            _current = _current.Next;
            return _current != _head;
        }

        public void Dispose()
        {
            _head = null;
            _current = null;
        }
    }

    #endregion

}
