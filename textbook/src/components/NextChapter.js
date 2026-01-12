import React from 'react';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './NextChapter.module.css';

const NextChapter = ({ nextUrl, nextTitle }) => {
  const location = useLocation();
  const { siteConfig } = useDocusaurusContext();

  if (!nextUrl || !nextTitle) {
    return null;
  }

  return (
    <div className={styles.nextChapterContainer}>
      <div className={styles.nextChapterCard}>
        <div className={styles.nextChapterHeader}>
          <span className={styles.bookIcon}>📖</span>
          <h3>Continue Learning</h3>
        </div>
        <p>Continue to the next chapter in {siteConfig.title}</p>
        <Link to={nextUrl} className={styles.nextChapterButton}>
          {nextTitle} →
        </Link>
      </div>
    </div>
  );
};

export default NextChapter;