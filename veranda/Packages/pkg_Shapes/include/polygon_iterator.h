//! \file
#pragma once

#include <QPolygonF>

#include <iterator>

/*!
 * \brief Mutable ForwardIterator which wraps a QPolygonF iterator
 * Presents a QPolygonF iterator as an iterator to an array of the form x, y, x, y...
 * to flatten out a '2d' QPolygonF
 *
 * Conforms as well as I can tell to the std::ForwardIterator spec, as well as InputIterator
 * and OutputIterator, which means it can be used with psimpl algorithms
 */
class PolygonIterator
{
    //! Iterator of points
    QPolygonF::iterator _parentIter;

    //! Record of which part of the current point is accessed
    uint8_t _index = 0;

public:
    //! Iterator difference trait type
    using difference_type = long;

    //! Iterator value trait type
    using value_type = double;

    //! Iterator pointer trait type
    using pointer = double*;

    //! Iterator reference trait type
    using reference = double&;

    //! Iterator category trait type
    using iterator_category = std::forward_iterator_tag;

    /*!
     * \brief Constructs a PolygonIterator pointing to a specific coordinate of a specific point in a QPolygonF
     * Default constructor points to QPolygonF::iterator()
     * \param[in] it Iterator in QPolygonF to point to
     * \param[in] ind index within the point to point to (must be 0 or 1)
     */
    PolygonIterator(QPolygonF::iterator it = QPolygonF::iterator(), uint8_t ind=0) : _parentIter(it), _index(ind % 2) {}

    /*!
     * \brief Copy Constructor of PolygonIterator
     * \param[in] other PolygonIterator to copy
     */
    PolygonIterator(PolygonIterator& other) : _parentIter(other._parentIter), _index(other._index) {}

    /*!
     * \brief Equality check; returns true when both the parent iterators are equal and the coodinate indexes are equal
     * \param[in] other PolygonIterator to compare
     * \return boolean
     */
    bool operator==(const PolygonIterator& other) const {return _parentIter == other._parentIter && _index == other._index;}

    /*!
     * \brief Inequality check; returns true when both the parent iterators are not equal or the coodinate indexes are not equal
     * \param[in] other PolygonIterator to compare
     * \return boolean
     */
    bool operator!=(const PolygonIterator& other) const {return _parentIter != other._parentIter || _index != other._index;}

    /*!
     * \brief Post-Increment operator
     * Increments the iterator and returns the value of the iterator previous to the increment
     * \return Iterator before increment
     */
    PolygonIterator operator++(int){
        PolygonIterator out = (_index ? PolygonIterator(_parentIter + 1, 0) : PolygonIterator(_parentIter, _index+1));
        if(((++_index) %= 2) == 0) _parentIter++;
        return out;
    }

    /*!
     * \brief Post-Increment operator
     * Increments the iterator and returns a reference to it
     * \return Iterator before increment
     */
    PolygonIterator& operator++() {if(((++_index) %= 2) == 0) _parentIter++; return *this;}

    /*!
     * \brief Dereference operator
     * \return A reference to the indexed coordinate of the current point
     */
    reference operator*() {return _index ? _parentIter->ry() : _parentIter->rx();}

    /*!
     * \brief Getter for parent iterator object
     * \return Copy of the current QPolygonF::iterator held
     */
    QPolygonF::iterator parentIterator(){ return _parentIter; }
};
