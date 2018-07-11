//! \file
#pragma once

#include <QPolygonF>

#include <iterator>

class PolygonIterator
{
    //! Iterator of points
    QPolygonF::iterator _parentIter;

    //! Record of which part of the current point is accessed
    int _index = 0;

public:
    // iterator traits
    using difference_type = long;
    using value_type = double;
    using pointer = double*;
    using reference = double&;
    using iterator_category = std::forward_iterator_tag;

    PolygonIterator(QPolygonF::iterator it = QPolygonF::iterator(), int ind=0) : _parentIter(it), _index(ind) {}
    PolygonIterator(PolygonIterator& other) : _parentIter(other._parentIter), _index(other._index) {}

    bool operator==(const PolygonIterator& other) const {return _parentIter == other._parentIter && _index == other._index;}
    bool operator!=(const PolygonIterator& other) const {return _parentIter != other._parentIter || _index != other._index;}

    PolygonIterator operator++(int){
        PolygonIterator out = (_index ? PolygonIterator(_parentIter + 1, 0) : PolygonIterator(_parentIter, _index+1));
        if(((++_index) %= 2) == 0) _parentIter++;
        return out;
    }
    PolygonIterator& operator++() {if(((++_index) %= 2) == 0) _parentIter++; return *this;}

    reference operator*() {return _index ? _parentIter->ry() : _parentIter->rx();}

    QPolygonF::Iterator parentIterator(){ return _parentIter; }
};
