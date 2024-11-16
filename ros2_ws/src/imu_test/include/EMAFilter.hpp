#ifndef EMA_FILTER_HPP
#define EMA_FILTER_HPP

class EMAFilter {
    private:
        double alpha;
        bool initialized;
    
    public:
        EMAFilter(double alpha);
        EMAFilter();
        void reset();
        void setAlpha(double alpha);
        void setInitial(double initial);
        double filter(double current);
        double last;
};

#endif