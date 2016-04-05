/*
 * Modified version of:
 *
 * @file htmlDoc.h
 * @brief Simple HTML document writer and SVG drawer
 * @author Pierre MOULON
 *
 * Copyright (c) 2011, 2012, 2013 Pierre MOULON
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _HTMLDOC_H_
#define _HTMLDOC_H_


// I N C L U D E S /////////////////////////////////////////////////


// D E F I N E S ///////////////////////////////////////////////////

#define JSXCHART_BORDER 0.2f


// S T R U C T S ///////////////////////////////////////////////////

namespace HTML {

inline const std::string htmlMarkup(const std::string& markup, const std::string& text) {
	std::ostringstream os;
	os << '<'<< markup<<'>' << text << "</"<<markup<<'>' <<"\n";
	return os.str();
}
inline const std::string htmlMarkup(const std::string& markup, const std::string& attributes, const std::string& text) {
	std::ostringstream os;
	os << '<'<<markup<<' '<<attributes<<'>' << text << "</"<<markup<<'>' <<"\n";
	return os.str();
}
inline const std::string htmlOpenMarkup(const std::string& markup, const std::string& attributes) {
	std::ostringstream os;
	os << '<'<<markup<<' '<<attributes<<"/>" <<"\n";
	return os.str();
}

inline const std::string htmlComment(const std::string& text) {
	std::ostringstream os;
	os << "<!-- "<< text << " -->" << "\n";
	return os.str();
}

/// Return a chain in the form attributes="val"
template<typename T>
inline const std::string quotedAttributes(const std::string& attributes, const T & val) {
	std::ostringstream os;
	os << attributes << "='" << val << '\'';
	return os.str();
}

/// Return a chain of the value T
template<typename T>
inline const std::string toString(const T& val) {
	std::ostringstream os;
	os << val;
	return os.str();
}

class htmlDocumentStream
{
public:
	htmlDocumentStream(const std::string& title) {
		htmlStream << "\n";
		htmlStream << htmlMarkup("head",
			"\n"
			"<link rel='stylesheet' type='text/css' href='http://jsxgraph.uni-bayreuth.de/distrib/jsxgraph.css' />\n"
			"<script type='text/javascript' src='http://jsxgraph.uni-bayreuth.de/distrib/jsxgraphcore-0.96.js'></script>\n");
		htmlStream << htmlMarkup("title",title);
	}

	htmlDocumentStream(const std::string& title,
		const std::vector<std::string>& vec_css,
		const std::vector<std::string>& vec_js)
	{
		htmlStream << "\n<head>\n";
		htmlStream << htmlMarkup("title",title);
		// CSS and JS resources
		for (std::vector<std::string>::const_iterator iter = vec_css.begin(); iter != vec_css.end(); ++iter)
			htmlStream << "<link rel='stylesheet' type='text/css' href='" << *iter <<"' />\n";
		for (std::vector<std::string>::const_iterator iter = vec_js.begin(); iter != vec_js.end(); ++iter)
			htmlStream << "<script type='text/javascript' src='" << *iter <<"'> </script>\n";
		htmlStream << "</head>\n";
	}

	void pushInfo(const std::string& text) {
		htmlStream << text;
	}

	std::string getDoc() {
		return htmlMarkup("html", htmlStream.str());
	}

	std::ostringstream htmlStream;
};


/// Class to draw with the JSXGraph library in HTML page.
class JSXGraphWrapper
{
public:
	typedef float TRANGE;
	typedef std::pair< std::pair<TRANGE,TRANGE>, std::pair<TRANGE,TRANGE> > RANGE;

	JSXGraphWrapper() {
		cpt = 0;
	}

	void reset() {
		stream.str("");
		stream.precision(4);
		//stream.setf(std::ios::fixed,std::ios::floatfield);
		cpt = 0;
	}

	void init(unsigned W, unsigned H, LPCTSTR szGraphName=NULL) {
		reset();
		std::string strGraphName;
		if (szGraphName == NULL) {
			strGraphName = SEACAVE::Util::getUniqueName();
			szGraphName = strGraphName.c_str();
		}
		stream <<
			"\n"
			"<div id='" << szGraphName << "' class='jxgbox' style='width:"<< W << "px; height:" << H <<"px;'></div>\n"
			"<script type='text/javascript'>\n"
			"var board = JXG.JSXGraph.initBoard('"<< szGraphName <<"', {axis:true,showCopyright:false});\n"
			"board.suspendUpdate();\n";
	}

	template<typename VECTX, typename VECTY, typename STR>
	void addXYChart(const VECTX& vec_x, const VECTY& vec_y, const STR& stype, LPCTSTR sface=_T("o"), LPCTSTR sizeStroke=_T("2"), LPCTSTR colFill=_T("#0077cc"), LPCTSTR colStroke=_T("#0044ee")) {
		typedef typename VECTX::value_type TX;
		typedef typename VECTY::value_type TY;
		size_t index0 = cpt++;
		size_t index1 = cpt++;
		stream << "var data"<< index0<<"= [";
		std::copy(vec_x.begin(), vec_x.end(), std::ostream_iterator<TX>(stream, ","));
		stream << "];\n";
		stream << "var data"<< index1<<"= [";
		std::copy(vec_y.begin(), vec_y.end(), std::ostream_iterator<TY>(stream, ","));
		stream << "];\n";
		std::ostringstream osData;
		osData << "[data" <<index0<<","<<"data"<<index1<<"]";
		stream << "board.createElement('chart', " <<osData.str()
			<< ", {chartStyle:'"<< stype << "',labels:" << osData.str() << ",face:'" << sface
			<< "',strokeWidth:" << sizeStroke << ",fillColor:'" << colFill << "',highlightStrokeColor:'" << colStroke << "',fixed:true});\n";
	}

	template<typename VECTY, typename STR>
	void addYChart(const VECTY& vec_y, const STR& stype, LPCTSTR sface=_T("o"), LPCTSTR sizeStroke=_T("2"), LPCTSTR colFill=_T("#0077cc"), LPCTSTR colStroke=_T("#0044ee")) {
		typedef typename VECTY::value_type TY;
		size_t index0 = cpt++;
		stream << "var data"<< index0<<"= [";
		std::copy(vec_y.begin(), vec_y.end(), std::ostream_iterator<TY>(stream, ","));
		stream << "];\n";
		stream << "board.createElement('chart', " << "data"<<index0
			<< ", {chartStyle:'" << stype << "',labels:" << "data"<<index0 << ",face:'" << sface
			<< "',strokeWidth:" << sizeStroke << ",fillColor:'" << colFill << "',highlightStrokeColor:'" << colStroke << "',fixed:true});\n";
	}

	template<typename TX, typename TY>
	void addLine(TX x0, TY y0, TX x1, TY y1, LPCTSTR color=_T("#00ff00")) {
		size_t index0 = cpt++;
		size_t index1 = cpt++;
		stream <<
			"var p"<<index0<<" = board.create('point',["<<x0<<","<<y0<<"], {fixed:true});\n"
			"var p"<<index1<<" = board.create('point',["<<x1<<","<<y1<<"], {fixed:true});\n"
			"var li = board.create('line',[p"<<index0<<",p"<<index1<<"], "
			"{strokeColor:'"<< color <<"',strokeWidth:2});\n";
	}

	void setViewport(const RANGE& range) {
		stream
			<< "board.setBoundingBox(["
			<< range.first.first << ","<< range.second.second <<","
			<< range.first.second << ","<< range.second.first <<"]);\n";
	}

	void UnsuspendUpdate() {
		stream << "board.unsuspendUpdate();\n";
	}
	void close() {
		stream << "</script>\n";
	}

	std::string toStr() const {
		return stream.str();
	}

	template<typename TX, typename TY>
	static inline RANGE autoViewport(TX maxValX, TY maxValY, TX minValX, TY minValY) {
		//Use the value with a little margin
		const TX rangeX = maxValX-minValX;
		const TY rangeY = maxValY-minValY;
		return std::make_pair(
			std::make_pair(-JSXCHART_BORDER*rangeX+minValX,JSXCHART_BORDER*rangeX+maxValX),
			std::make_pair(-JSXCHART_BORDER*rangeY+minValY,JSXCHART_BORDER*rangeY+maxValY));
	}
	template<typename VECTX, typename VECTY>
	static RANGE autoViewport(const VECTX& vec_x, const VECTY& vec_y) {
		typedef typename VECTX::value_type TX;
		typedef typename VECTY::value_type TY;
		if (vec_x.empty() || vec_y.empty() || vec_x.size() != vec_y.size())
			return RANGE();
		//For X values
		const TX minValX = *std::min_element(vec_x.begin(), vec_x.end());
		const TX maxValX = *std::max_element(vec_x.begin(), vec_x.end());
		//For Y values
		const TY minValY = *std::min_element(vec_y.begin(), vec_y.end());
		const TY maxValY = *std::max_element(vec_y.begin(), vec_y.end());
		return autoViewport(maxValX, maxValY, minValX, minValY);
	}
	template<typename T, typename VECTY>
	static RANGE autoViewport(const VECTY& vec_y, bool bForceY0=true) {
		typedef T TX;
		typedef typename VECTY::value_type TY;
		if (vec_y.empty())
			return RANGE();
		//For X values
		const TX minValX = TX(0);
		const TX maxValX = static_cast<TX>(vec_y.size());
		//For Y values
		const TY minValY = (bForceY0 ? TY(0) : *std::min_element(vec_y.begin(), vec_y.end()));
		const TY maxValY = *std::max_element(vec_y.begin(), vec_y.end());
		return autoViewport(maxValX, maxValY, minValX, minValY);
	}

	std::ostringstream stream;
	size_t cpt; //increment for variable
};
/*----------------------------------------------------------------*/

} // namespace HTML



namespace SVG {

/// Basic SVG style
class svgStyle
{
public:
	svgStyle():_sFillCol(""), _sStrokeCol("black"), _sToolTip(""), _fillOpacity(1.f), _strokeW(1.f), _strokeOpacity(1.f) {}

	// Configure fill color
	svgStyle& fill(const std::string& col, float opacity = 1.f)
	{ _sFillCol = col; _fillOpacity = opacity; return *this; }

	// Configure stroke color and width
	svgStyle& stroke(const std::string& col, float witdh = 1.f, float opacity = 1.f)
	{ _sStrokeCol = col; _strokeW = witdh; _strokeOpacity = opacity; return *this; }

	// Configure with no stroke
	svgStyle& noStroke()
	{ _sStrokeCol = "";  _strokeW = 0.f; _strokeOpacity = 0.f; return *this; }

	// Configure tooltip
	svgStyle& tooltip(const std::string& sTooltip)
	{ _sToolTip = sTooltip; return *this; }

	const std::string getSvgStream() const {
		std::ostringstream os;
		if (!_sStrokeCol.empty()) {
			os << " stroke='" << _sStrokeCol << "' stroke-width='" << _strokeW << "'";
			if (_strokeOpacity < 1)
				os << " stroke-opacity='" << _strokeOpacity << "'";
		}
		if (!_sFillCol.empty()) {
			os << " fill='" << _sFillCol << "'";
			if (_fillOpacity < 1)
				os << " fill-opacity='" << _fillOpacity << "'";
		} else {
			os << " fill='none'";
		}
		if (!_sToolTip.empty()) {
			os << " tooltip='enable'>" << "<title>" << _sToolTip << "</title>";
		}
		return os.str();
	}

	bool bTooltip() const { return !_sToolTip.empty();}

	std::string _sFillCol, _sStrokeCol, _sToolTip;
	float _fillOpacity, _strokeW, _strokeOpacity;
};


/// Basic class to handle simple SVG draw.
/// You can draw line, square, rectangle, text and image (xlink)
class svgDrawer
{
public:
	///Constructor
	svgDrawer(size_t W = 0, size_t H = 0) {
		svgStream <<
			"<?xml version='1.0' standalone='yes'?>\n"
			"<!-- SVG graphic -->\n"
			"<svg";

		if (W > 0 && H > 0)
			svgStream <<
				" width='" << W << "px' height='"<< H << "px'"
				" preserveAspectRatio='xMinYMin meet'"
				" viewBox='0 0 " << W << ' ' << H <<"'";

		svgStream <<
			" xmlns='http://www.w3.org/2000/svg'"
			" xmlns:xlink='http://www.w3.org/1999/xlink'"
			" version='1.1'>\n";
	}
	///Circle draw -> x,y position and radius
	void drawCircle(float cx, float cy, float r, const svgStyle& style) {
		svgStream
			<< "<circle cx='" << cx << "'" << " cy='" << cy << "'"
			<< " r='" << r << "'"
			<< style.getSvgStream() + (style.bTooltip() ? "</circle>\n" : "/>\n");
	}
	///Line draw -> start and end point
	void drawLine(float ax, float ay, float bx, float by, const svgStyle& style) {
		svgStream
			<< "<line x1='"<<ax<< "' y1='"<<ay<< "' x2='"<<bx<< "' y2='"<<by<< "'"
			<< style.getSvgStream() +  (style.bTooltip() ? "</line>\n" : "/>\n");
	}

	///Reference to an image (path must be relative to the SVG file)
	void drawImage(const std::string& simagePath, int W, int H,
		int posx = 0, int posy = 0, float opacity =1.f)
	{
		svgStream <<
			"<image x='"<< posx << "'" << " y='"<< posy << "'"
			" width='"<< W << "px'" << " height='"<< H << "px'"
			" opacity='"<< opacity << "'"
			" xlink:href='" << simagePath << "'/>\n";
	}

	///Square draw -> x,y position and size
	void drawSquare(float cx, float cy, float W, const svgStyle& style) {
		drawRectangle(cx, cy, W, W, style);
	}

	///Circle draw -> x,y position and width and height
	void drawRectangle(float cx, float cy, float W, float H, const svgStyle& style) {
		svgStream
			<< "<rect x='" << cx << "'"
			<< " y='" << cy << "'"
			<< " width='" << W << "'"
			<< " height='" << H << "'"
			<< style.getSvgStream() + (style.bTooltip() ? "</rect>\n" : "/>\n");
	}

	///Text display -> x,y position, font size
	void drawText(float cx, float cy, const std::string& stext, const std::string& scol = "", const std::string& sattr = "", float fontSize = 1.f) {
		svgStream << "<text" << " x='" << cx << "'" << " y='" << cy << "'"
			<< " font-size='" << fontSize << "'";
		if (!sattr.empty())
			svgStream << ' ' << sattr;
		if (!scol.empty())
			svgStream << " fill='" << scol << "'";
		svgStream << ">" << stext << "</text>\n";
	}
	template< typename DataInputIteratorX, typename DataInputIteratorY>
	void drawPolyline(DataInputIteratorX xStart, DataInputIteratorX xEnd,
		DataInputIteratorY yStart, DataInputIteratorY /*yEnd*/,
		const svgStyle& style)
	{
		svgStream << "<polyline points='";
		DataInputIteratorY itery = yStart;
		for(DataInputIteratorX iterx = xStart;
			iterx != xEnd; std::advance(iterx, 1), std::advance(itery, 1))
		{
			svgStream << *iterx << ',' << *itery << ' ';
		}
		svgStream << "'" << style.getSvgStream() + (style.bTooltip() ? "</polyline>\n" : "/>\n");
	}

	///Close the svg tag.
	std::ostringstream& closeSvgFile() {
		svgStream << "</svg>";
		return svgStream;
	}

	std::ostringstream svgStream;
};

/// Helper to draw a SVG histogram
/// ____
/// |  |   ___ |
/// |  |__|  | |
/// |  |  |  | |
/// -----------|
struct svgHisto
{
	template<typename VECT>
	std::string draw(const VECT& vec_value,
		const std::pair<float, float>& range,
		float W, float H)
	{
		if (vec_value.empty())
			return "";

		//-- Max value
		typedef typename VECT::value_type T;
		const T maxi = *max_element(vec_value.begin(), vec_value.end());
		const size_t n = vec_value.size();

		const float scaleFactorY = H / static_cast<float>(maxi);
		const float scaleFactorX = W / static_cast<float>(n);

		svgDrawer svgStream;

		for (typename VECT::const_iterator iter = vec_value.begin(); iter != vec_value.end(); ++iter)
		{
			const T dist = std::distance(vec_value.begin(), iter);
			const T& val = *iter;
			std::ostringstream os;
			os << '(' << range.first + dist/float(n) * (range.second-range.first) << ',' << val << ')';
			svgStyle style = svgStyle().fill("blue").stroke("black", 1.0).tooltip(os.str());
			svgStream.drawRectangle(
				scaleFactorX * dist, H-val * scaleFactorY,
				scaleFactorX, val * scaleFactorY,
				style);
			//_________
			//|       |_________
			//|       ||       |
			//|       ||       |
			//|       ||       |
			//0    sFactorX  2*sFactorX
		}
		svgStyle styleAxis = svgStyle().stroke("black", 1.0f);
		// Draw X Axis
		svgStream.drawText(.05f*W, 1.2f*H, HTML::toString(range.first), "black", "", .1f*H);
		svgStream.drawText(W, 1.2*H, HTML::toString(range.second), "black", "", .1f*H);
		svgStream.drawLine(0, 1.1f*H, W, 1.1f*H, styleAxis);
		// Draw Y Axis
		svgStream.drawText(1.2f*W, .1f*H, HTML::toString(maxi), "black", "", .1f*H);
		svgStream.drawText(1.2f*W, H, "0", "black", "", .1f*H);
		svgStream.drawLine(1.1f*W, 0, 1.1f*W, H, styleAxis);

		return svgStream.closeSvgFile().str();
	}
};
/*----------------------------------------------------------------*/

} // namespace SVG

#endif // _HTMLDOC_H_
