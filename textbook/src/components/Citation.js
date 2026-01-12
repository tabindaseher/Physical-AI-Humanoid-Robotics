import React from 'react';
import styles from './Citation.module.css';

const Citation = ({ id, authors, year, title, journal, volume, issue, pages, doi, url }) => {
  const renderCitation = () => {
    let citationText = '';

    // Format authors
    if (authors && authors.length > 0) {
      if (authors.length === 1) {
        citationText += `${authors[0]}. `;
      } else if (authors.length === 2) {
        citationText += `${authors[0]} & ${authors[1]}. `;
      } else {
        citationText += `${authors[0]} et al. `;
      }
    }

    // Add year
    if (year) {
      citationText += `(${year}). `;
    }

    // Add title
    if (title) {
      citationText += `${title} `;
    }

    // Add journal information
    if (journal) {
      citationText += italicize(journal);
      if (volume) {
        citationText += `, ${bold(volume)}`;
      }
      if (issue) {
        citationText += `(${issue})`;
      }
      if (pages) {
        citationText += `, ${pages}.`;
      } else {
        citationText += '.';
      }
    } else {
      citationText += '.';
    }

    // Add DOI if available
    if (doi) {
      citationText += ` https://doi.org/${doi}`;
    }

    return citationText;
  };

  const italicize = (text) => {
    return <em>{text}</em>;
  };

  const bold = (text) => {
    return <strong>{text}</strong>;
  };

  const citationText = renderCitation();

  return (
    <div className={styles.citation}>
      <span className={styles.citationText}>
        {Array.isArray(citationText) ? citationText : renderCitation()}
      </span>
    </div>
  );
};

// APA citation component for inline citations
export const InlineCitation = ({ id, authors, year }) => {
  if (authors && authors.length > 0) {
    let authorText = '';
    if (authors.length === 1) {
      authorText = authors[0].split(',')[0]; // Get last name
    } else if (authors.length === 2) {
      authorText = `${authors[0].split(',')[0]} & ${authors[1].split(',')[0]}`;
    } else {
      authorText = `${authors[0].split(',')[0]} et al.`;
    }
    return <span className={styles.inlineCitation}>({authorText}, {year})</span>;
  }
  return <span className={styles.inlineCitation}>({year})</span>;
};

// Bibliography component to list all references
export const Bibliography = ({ references }) => {
  return (
    <div className={styles.bibliography}>
      <h3>References</h3>
      {references && references.map((ref, index) => (
        <div key={index} className={styles.bibliographyItem}>
          <Citation {...ref} />
        </div>
      ))}
    </div>
  );
};

export default Citation;